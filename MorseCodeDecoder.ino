/*
  MorseCodeDecoder6.ino
  Code by by LINGIB
  https://www.instructables.com/member/lingib/instructables/
  Last update 22 July 2020
  Tim Hatch W0AHO May 1st 2022

  This decoder features
   - Goertzel bandpass filter on 930Hz
   - Exact Blackman window
   - binary morse tree
   - noise_blanker
   - auto-speed tracking
   - calculates a new speed reference after each symbol based on maximum and minimum tone durations
   - extremely tolerant to "bad" sending

  The following characters are recognised:
   - [A..Z]
   - [0..9]
   - [. , ? ' ! / ( ) & : ; = + - _ " @]

  Morse filter algorithm and binary tree by LINGIB (callsign ZL2AVF)

  Goertzel filter algorithm from
  https://www.embedded.com/the-goertzel-algorithm/
  https://courses.cs.washington.edu/courses/cse466/12au/calendar/Goertzel-EETimes.pdf
  added Teensy ADC from https://github.com/pedvide/ADC W0AHO Tim Hatch
  -------------
  TERMS OF USE:
  -------------
  The software is provided "AS IS", without any warranty of any kind, express or implied,
  including but not limited to the warranties of mechantability, fitness for a particular
  purpose and noninfringement. In no event shall the authors or copyright holders be liable
  for any claim, damages or other liability, whether in an action of contract, tort or
  otherwise, arising from, out of or in connection with the software or the use or other
  dealings in the software.
*/

//  
#include <ADC.h>
ADC *adc = new ADC();
const short Buzzer = 4;                         // for morse key
const short LED = 7;                            // signal/tuning indicator
// ----- TFT shield definitions
#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"
#define TFT_CS    8                             // chip select | slave select
#define TFT_RST   9                             // reset
#define TFT_DC    10                            // data/command | register select | A0 (address select)
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);

// ----- Arduino connections
#define Morse_input A0                          // morse input

// ----- morse decoder
/*
  Morse comprises dots and dashes.
   - The length of a dot is one unit.
   - A dash is three units.
   - The space between parts of the same letter is one unit.
   - The space between letters is three units.
   - The space between words is 7 units.
   - A word is defined as fifty units
   - The word PARIS is exactly 50 units
   - 10 WPM (words per minute) is 10*50 = 500 units
     (which is the equivalent of sending the word PARIS
     ten times in one minute)
   - WPM = 1200/dot-width (mS)

  ----------------------------------------------
  Dot & dash widths (mS) for various WPM follow
  ----------------------------------------------
  WPM  Dot   Dash        WPM   Dot   Dash
  1    1200  3600        11    109   327
  2    600   1800        12    100   300    <=== 200 is midway between a dot/dash at 12 WPM
  3    400   1200        13    92    276
  4    300   900         14    86    257
  5    240   720         15    18    240
  6    200   600         16    75    225
  7    171   514         17    71    211
  8    150   450         18    67    199
  9    133   400         19    63    189
  10   120   360         20    60    180
*/

bool Debug = false;                             // use to set Threshold while viewing your Serial Monitor
static const float Threshold = 2000;            // threshold for Magnitude = sqrt(Magnitude_squared);
short Noise_blanker = 1;                        // number of confirmations ... adds 8mS per loop (0 = no noise blanking

unsigned long Start_reference = 200;            // choose value midway between dot and dash at target speed
unsigned long Reference = Start_reference;
unsigned long Leading_edge;                     // used to calculate tone duration
unsigned long Trailing_edge;                    // used to calculate tone duration
unsigned long Tone_duration[6];                 // store tone durations for entire letter here
short Tone_index = 0;
unsigned long Tone_max = 0L;
unsigned long Tone_min = 9999L;

unsigned long Duration;                         // used for calculating rolling dot/dash averages;
unsigned long Average;

unsigned long Letter_start;                     // used for calculating WPM (words per minute)
short Symbol_count = 0;
short WPM;

bool Started = false;                           // decoder logic
bool Measuring = false;
bool Tone = false;

// ----- binary morse tree
const char Symbol[] =
{
  ' ', '5', ' ', 'H', ' ', '4', ' ', 'S',       // 0
  ' ', ' ', ' ', 'V', ' ', '3', ' ', 'I',       // 8
  ' ', ' ', ' ', 'F', ' ', ' ', ' ', 'U',       // 16
  '?', ' ', '_', ' ', ' ', '2', ' ', 'E',       // 24
  ' ', '&', ' ', 'L', '"', ' ', ' ', 'R',       // 32
  ' ', '+', '.', ' ', ' ', ' ', ' ', 'A',       // 40
  ' ', ' ', ' ', 'P', '@', ' ', ' ', 'W',       // 48
  ' ', ' ', ' ', 'J', '\'', '1', ' ', ' ',      // 56
  ' ', '6', '-', 'B', ' ', '=', ' ', 'D',       // 64
  ' ', '/', ' ', 'X', ' ', ' ', ' ', 'N',       // 72
  ' ', ' ', ' ', 'C', ';', ' ', '!', 'K',       // 80
  ' ', '(', ')', 'Y', ' ', ' ', ' ', 'T',       // 88
  ' ', '7', ' ', 'Z', ' ', ' ', ',', 'G',       // 96
  ' ', ' ', ' ', 'Q', ' ', ' ', ' ', 'M',       // 104
  ':', '8', ' ', ' ', ' ', ' ', ' ', 'O',       // 112
  ' ', '9', ' ', ' ', ' ', '0', ' ', ' '        // 120
};
short Index = 63;                               // point to middle of symbol[] array
short Offset = 32;                              // amount to shift index for first dot/dash
short Count = 6;                                // Height we are up the tree

// ----- Goertzel filter
/*
  The Goertzel algorithm is explained in detail at:
  https://www.embedded.com/the-goertzel-algorithm/
  https://courses.cs.washington.edu/courses/cse466/12au/calendar/Goertzel-EETimes.pdf
  http://www.mstarlabs.com/dsp/goertzel/goertzel.html

  -----------
  Notes:
  -----------
  (1)
  Google says:
  "For a 16 MHz Arduino the ADC clock is set to 16 MHz/128 = 125 KHz.
  Each conversion in AVR takes 13 ADC clocks so 125 KHz /13 = 9615 Hz.
  That is the maximum possible sampling rate, but the actual sampling
  rate in your application depends on the interval between successive
  conversions calls."

  Actual time to take 48 samples (code lines 569,570,571) is 5386uS
  which equates to 8192 samples/sec

  (2)
  Maximum target frequency is 8912/2 = 4456Hz (Nyquist)

  (3)
   F =  Fsample * K / N

   where:
   F = center frequency
   K = integer constant
   N = number of samples
   Fsample = samples per second

  (4)
   "bin_width" = Sampling_freq / N

  (5)
   "Exact Blackman" window = 0.426591 - 0.496561 * cos((2.0 * PI * i) / n) + 0.076848 * cos((4.0 * PI * i) / n)
*/

static const float Target_freq = 930.0;                                     // see above notes
static const float Sampling_freq = 8912.0;                                  // see above notes
static const short N = 48;                                                  // number of samples
static const short K = round(N * Target_freq / Sampling_freq);              // constant
static const float Omega = (2.0 * PI * K) / N;
static const float Sine =  sin(Omega);
static const float Cosine = cos(Omega);
static const float Coeff = 2.0 * Cosine;

float Q0;
float Q1;
float Q2;

short TestData[N];                                                          // samples stored here
float Blackman[N];                                                          // "Exact Blackman" window values stored here
float Magnitude;                                                            // signal level

// ----------------------------
// setup()
// ----------------------------
void setup() {

  // ----- configure serial port
  Serial.begin(115200);

  // ----- configure TFT display
  tft.begin();
  tft.setRotation(3);                             // set graphics page to landscape
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);                             // 10*16 pixels per character
  tft.setCursor(10, 10);                          // 10 pixel margins

  // ----- configure morse decoder/sender
  pinMode(Morse_input, INPUT);                    // morse input
  tone(Buzzer, 930);                              // buzzer output
  /*
       930Hz used as 1-inch speaker output level drops rapidly below 900 Hz
  */

  // ----- configure LED
  pinMode(LED, OUTPUT);                           // signal/tuning indicator
  digitalWrite(LED, LOW);

  // ----- create "Exact Blackman" lookup table
  for (short i = 0; i < N; i++) {
    Blackman[i] = (0.426591 - 0.496561 * cos((2.0 * PI * i) / N) + 0.076848 * cos((4.0 * PI * i) / N));
  }

  Started = false;
  Measuring = false;
  adc_init(); 
}

// ----------------------------
// loop()
// ----------------------------
void loop() {

  // ----- check for tone
  sample();

  // ----- debug
  if (Debug) {
    Serial.println(Magnitude);
    delay(250);
  }

  /////////////////////////////////////////////////
  //  Wait for initial leading_edge
  /////////////////////////////////////////////////

  // ----- Wait for initial leading_edge
  if (!Started && !Measuring && Tone) {                         // initial leading edge detected
    Leading_edge = millis();
    Letter_start = Leading_edge;                                // record first edge... used to detect timeout

    // ----- insert word space
    /*
      -------
      Notes:
      -------
       - symbol gap = 1 unit
       - Reference  = 2 units
       - Letter end = 3 units
       - Word end   = 7 units
       - a word space is inserted at startup
    */

    if ((millis() - Trailing_edge) > Reference * 3) {           // detect word end
      Serial.print(' ');                                        // if so insert a space
      tft_print(' ');
    }

    // ----- now look for a trailing edge
    Started = true;
    Measuring = true;
  }

  /////////////////////////////////////////////////
  //  Wait for trailing_edge
  /////////////////////////////////////////////////

  // ----- Wait for trailing leading_edge
  if (Started && Measuring && !Tone) {                          // trailing edge detected

    // ----- record time of last trailing edge
    Trailing_edge = millis();                                   // needed for detecting word gap

    // ----- calculate tone duration
    Duration = Trailing_edge - Leading_edge;

    // ----- use default reference until we have received both a dot and a dash
    if ((Tone_min < 9999) && (Tone_max > 0) && (Tone_min != Tone_max)) {

      // ---- perform rolling dot/dash averages
      if (Duration < Reference) {

        // ----- we have a dot
        Tone_min = (Tone_min + Duration) / 2;
        Tone_duration[Tone_index] = Tone_min;                    // record average Tone_min tone duration
        Tone_index++;
        Reference = (Tone_min + Tone_max) / 2;

      } else {

        // ----- we have a dash
        Tone_max = (Tone_max + Duration) / 2;
        Tone_duration[Tone_index] = Tone_max;                    // record average Tone_min tone duration
        Tone_index++;
        Reference = (Tone_min + Tone_max) / 2;
      }

    } else {

      // ----- save initial (raw) tones to array
      /*
         Use the default reference until both a dot and a dash have been received.
         Tone_min equals Tone_max after the first tone which means that we can't
         calculate a new reference using (Tone_min +Tone_max)/2 until we have
         received at least one dot and one dash
      */
      Tone_duration[Tone_index] = Duration;                       // record tone duration
      Tone_min = min(Tone_min, Tone_duration[Tone_index]);        // needed to calculate reference when decoding
      Tone_max = max(Tone_max, Tone_duration[Tone_index]);
      Tone_index++;
    }

    // ----- now look for a leading edge
    Started = true;
    Measuring = false;
  }

  /////////////////////////////////////////////////
  //  Wait for second and subsequent leading edges
  /////////////////////////////////////////////////
  /*
    -------
    Notes:
    -------
    - this space may never end (last letter)
    - it could be a gap between the letter elements
    - it could be a letter gap
    - it could be a word gap
    - dot is 1 unit
    - gap between letters is 1 unit
    - "reference" is 2 units
    - dash is 3 units
    - letter space is 3 units
    - word space is 7 units
  */

  // ----- measure space duration
  if (Started && !Measuring & Tone) {

    // ----- a leading edge has arrived
    if ((millis() - Trailing_edge) <  Reference) {          // detects gaps between letter elements
      Leading_edge = millis();

      // ----- now look for a trailing edge
      Started = true;
      Measuring = true;
    }
  }

  /////////////////////////////////////////////////
  //  Timeout
  /////////////////////////////////////////////////
  /*
     --------
     Notes:
     --------
     - calculates new "Reference" after each letter
     - old reference used if we encounter letters E,I,S,H,5,T,M,O,0 (all symbols the same !!)
     - moves the binary pointers
     - prints the letter
     - calculates the WPM (not used)
     - resets the points
  */

  // ----- a leading edge hasn't arrived
  if (Started && !Measuring & !Tone) {

    // ----- timeout
    if ((millis() - Trailing_edge) >  Reference) {

      // ----- calculate new reference
      if (Tone_max != Tone_min) {                          // use old reference if letter is all dots / dashes
        Reference = (Tone_max + Tone_min) / 2;
      }

      // ----- move the pointers
      for (short i = 0; i < Tone_index; i++) {
        (Tone_duration[i] < Reference) ? dot() : dash();
      }

      // ----- print letter
      Serial.print(Symbol[Index]);                         // print letter to Serial Monitor
      tft_print(Symbol[Index]);                            // print character to TFT display

      // ----- calculate WPM
      WPM = ((Symbol_count - 1) * 1200) / (Trailing_edge - Letter_start);

      // ----- reset pointers
      Index = 63;                                          // reset binary pointers
      Offset = 32;
      Count = 6;

      Tone_index = 0;                                     // reset decoder
      Symbol_count = 0;

      // ----- point to start
      Started = false;
      Measuring = false;
    }
  }
}

// ------------------------------
//   dot()
// ------------------------------
void dot() {

  // ----- we have a dot
  Index -=  Offset;                                       // move the pointer left
  Offset /= 2;                                            // prepare for next move
  Count--;
  Symbol_count += 2;                                      // 1*dot + 1*space

  // ----- error check
  if (Count < 0) {

    Index = 63;                                          // reset  binary pointers
    Offset = 32;
    Count = 6;

    Reference = Start_reference;                         // reset decoder
    Tone_min = 9999L;
    Tone_max = 0L;
    Tone_index = 0;
    Symbol_count = 0;
    Serial.println("error");

    // ----- point to start
    Started = false;
    Measuring = false;
  }
}

// ------------------------------
//   dash()
// ------------------------------
void dash() {

  // ----- we have a dash
  Index +=  Offset;                                       // move pointer right
  Offset /= 2;                                            // prepare for next move
  Count--;
  Symbol_count += 4;                                      // 3*dots + 1*space

  // ----- error check
  if (Count < 0) {

    Index = 63;                                          // reset  binary pointers
    Offset = 32;
    Count = 6;

    Reference = Start_reference;                         // reset decoder
    Tone_min = 9999L;
    Tone_max = 0L;
    Tone_index = 0;
    Symbol_count = 0;
    Serial.println("error");

    // ----- point to start
    Started = false;
    Measuring = false;
  }
}

// ------------------------------
//   tft_print(character);
// ------------------------------
/*
   Each line of screen characters are held in an array. The array contents are over-
   written each line which means that the array is holding the bottom screen line when
   the last character is printed. The display is then blanked and the array contents
   reprinted at the top. This gives the illusion that the screen has been "paged-up".
*/
void tft_print(char character) {

  // ----- locals
  static short y_axis = 10;                               // y-axis has 240 pixels
  static short character_count = 0;
  static char letters[25];                                // storage for one text line

  // ----- print character
  tft.print(character);
  letters[character_count] = character;                   // overwrites existing characters

  // ----- manage screen
  character_count++;

  // ----- line end?
  if (character_count > 24) {                             // 25 characters per line
    character_count = 0;

    // ----- point to next line
    y_axis += 25;                                         // line spacing added to 10*18 character
    tft.setCursor(10, y_axis);

    // ----- page end?
    if (y_axis > 220) {
      tft.fillScreen(ILI9341_BLACK);                      // clear screen

      // ----- point to start of line 1
      character_count = 0;                                // reset character pointer
      y_axis = 10;                                        // reset y_axis
      tft.setCursor(10, 10);

      // -----copy last line to first line
      for (short i = 0; i < 25; i++) {
        tft.print(letters[i]);                            // print last line of text
        character_count++;
      }

      // ----- point to start of second line
      character_count = 0;
      y_axis += 25;
      tft.setCursor(10, y_axis);
    }
  }
}

// --------------------------------
//  sample()
// --------------------------------
boolean sample() {

  /*
    Two integrators are used to suppress false triggers due to noise.
    The first integrator to reach Noise_blanker wins.
    Each Goertzel loop takes 8uS which means 1 confirmation requires 16uS.

    Sample time:
    0 = 8mS                                             // Noise_blanker = off
    1 = 15 mS                                           // time for 1 confirmation
    5 = 46 mS                                           // time for 5 confirmations
    10 = 86 mS                                          // time for 10 confirmations
  */

  // ----- Locals
  int no_tone_ = 0;                                     // Integrator
  int tone_ = 0;                                        // Integrator

  // ----- check for tone/no_tone
  /*
     goertzel() returns true if tone present otherwise false
  */
  while (1) {
    if (goertzel()) {
      // ----- tone
      no_tone_ = 0;                                     // empty opposite integrator
      tone_++;                                          // integrate
      if (tone_ > Noise_blanker) return true;           // no noiseblanking if Noise_blanker = 0;
    } else {
      // ----- no_tone
      tone_ = 0;                                        // empty opposite integrator
      no_tone_++;                                       // integrate
      if (no_tone_ > Noise_blanker) return false;       // no noiseblanking if Noise_blanker = 0;
    }
  }
}

// ----------------------------
//  Goertzel algorithm
// ----------------------------
boolean goertzel() {

  // ----- sample the input waveform
  /*
     Do not merge this loop with the loops below.
     The filter center frequency is lowered if you do.
  */
  for (short index = 0; index < N; index++) {
    TestData[index] = adc->adc0->analogRead(Morse_input);
  }

  // ----- apply Exact Blackman window
  for (short index = 0; index < N; index++) {
    TestData[index] *= Blackman[index];
  }

  // ----- Goertzel algorithm
  Q1 = 0;
  Q2 = 0;
  for (short index = 0; index < N; index++) {
    Q0 = Coeff * Q1 - Q2 + (float) TestData[index];
    Q2 = Q1;
    Q1 = Q0;
  }
  float Magnitude_squared = (Q1 * Q1) + (Q2 * Q2) - Q1 * Q2 * Coeff;
  Magnitude = sqrt(Magnitude_squared);

  // ----- apply threshold
  if (Magnitude > Threshold) {
    Tone = true;                                          // tone detected
    digitalWrite(LED, HIGH);
    return true;
  } else {
    Tone = false;                                         // space detected
    digitalWrite(LED, LOW);
    return false;
  }
}

void adc_init(void)
{
  adc->adc0->setReference(ADC_REFERENCE::REF_3V3);
  adc->adc1->setReference(ADC_REFERENCE::REF_3V3);
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_LOW_SPEED);     // Sampling speed, ADC_VERY_LOW_SPEED, ADC_LOW_SPEED, ADC_MED_SPEED, ADC_HIGH_SPEED or ADC_VERY_HIGH_SPEED
  adc->adc1->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_LOW_SPEED);
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_LOW_SPEED); // Conversion speed
  adc->adc1->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_LOW_SPEED);
  adc->adc0->setResolution(12);                                         // AD resolution, 12 bits
  adc->adc1->setResolution(12);
  adc->adc0->setAveraging(16);                                          // Averaging of 16 samples for better bit resolution, at
  adc->adc1->setAveraging(16);                                          // VERY_HIGH_SPEED  This results in each measurement taking 20us
}
