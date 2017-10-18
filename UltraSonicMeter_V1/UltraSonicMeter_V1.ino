#include <Arduino.h>
#include "FastLED.h"

// Distance for 100->0% 
#define DISTANCE_MIN_CM    30
#define DISTANCE_MAX_CM   160

// Loop update (seem fine with 500)
#define LOOP_UPDATE_MS    500 

// Default Mode Red/Green balance
#define LED_R_K  23  // Max 23 (255/11)
#define LED_G_K  18  // Max 23 (255/11)

/// !!! CAUTION THESE SETTINGS ARE HW RELATED !!!
#define USONIC_OUTPUT 2
#define USONIC_INPUT  3
#define USONIC_PULSE_US      25
#define USONIC_ECHO_MAX_US  (30*1000UL) // 30 ms: about 5.1 meters (with debounce)

#define WS2812B_OUTPUT  4
#define BP_INPUT        5

uint16_t Loop_Counter;
uint16_t Distance_CM;
uint32_t Loop_Ref_Ms;
uint8_t  LED_Hue;

typedef enum
{
  LED_MODE__DEFAULT = 0,
  LED_MODE__ONE_LED,
  LED_MODE__RAINBOW,
  LED_MODE__RAINBOW_GLITTER,
  LED_MODE__CONFETTI,
  LED_MODE__SINELON,
  LED_MODE__BPM,
  LED_MODE__JUGGLE,
  LED_MODE__COUNT
} LED_Mode_t;
LED_Mode_t LED_Mode;

void Debug_PrintMode()
{
  Serial.print("LED Mode is: "); Serial.print(LED_Mode); Serial.print(" - "); 
  switch(LED_Mode)
  {
    case LED_MODE__DEFAULT:         Serial.print("Default");          break;
    case LED_MODE__ONE_LED:         Serial.print("One Led");          break;
    case LED_MODE__RAINBOW:         Serial.print("RainBow");          break;
    case LED_MODE__RAINBOW_GLITTER: Serial.print("RainBow Glitter");  break;
    case LED_MODE__CONFETTI:        Serial.print("Confetti");         break;
    case LED_MODE__SINELON:         Serial.print("Sinelon");          break;
    case LED_MODE__BPM:             Serial.print("BPM");              break;
    case LED_MODE__JUGGLE:          Serial.print("Juggle");           break;
  }
  Serial.println(".");
}

typedef enum
{
  LED_BRIGHTNESS__LOW = 32,
  LED_BRIGHTNESS__MED = 48,
  LED_BRIGHTNESS__HIG = 64,
} LED_Brightness_t;
LED_Brightness_t LED_Brightness;

void Debug_PrintBrightness()
{
  Serial.print("LED Brightness is: "); Serial.print(LED_Brightness); Serial.print(" - "); 
  switch(LED_Brightness)
  {
    case LED_BRIGHTNESS__LOW: Serial.print("LOW");  break;
    case LED_BRIGHTNESS__MED: Serial.print("MED");  break;
    case LED_BRIGHTNESS__HIG: Serial.print("HIG");  break;
  }
  Serial.println(".");
}

// Definition des LEDs
#define LED_COUNT 12
CRGB LEDs[LED_COUNT];
void setup() 
{
  Serial.begin(115200); 
  Serial.println("Wait.");
  
  // Reset
  Loop_Counter = 0;
  Distance_CM = 0;
  LED_Mode = LED_MODE__DEFAULT;
  LED_Hue = 0;
  LED_Brightness = LED_BRIGHTNESS__LOW;

  // Wait Bootup confirmation time 
  delay(3000);
  
  // Init des LEDs de type "WS2812B" 
  FastLED.addLeds<WS2812B, WS2812B_OUTPUT, GRB>(LEDs, LED_COUNT);
  
  // set master brightness control
  FastLED.setBrightness(LED_Brightness);
  
  // Init
  pinMode(USONIC_OUTPUT, OUTPUT);
  pinMode(USONIC_INPUT, INPUT);  
  pinMode(BP_INPUT, INPUT_PULLUP);
  
  // Go !
  Serial.println("Ready.");
  Loop_Ref_Ms = millis();
  Debug_PrintMode();
}

uint16_t UltraSonic_Distance_CM()
{
  uint32_t EchoDurationUs = 0;
  
noInterrupts();
  
  // Send Pulse
  digitalWrite(USONIC_OUTPUT, HIGH); 
  delayMicroseconds(USONIC_PULSE_US);
  digitalWrite(USONIC_OUTPUT, LOW);
  
  // Wait Echo
  EchoDurationUs = pulseIn(USONIC_INPUT, HIGH);
 
interrupts();

  // Debug
  // Serial.print("EchoDurationUs: "); Serial.println(EchoDurationUs);
  
  // Convert µS into CM
  // (340.29 * 100) / (1000 * 1000 * 2) ~ 17 / 1000
  if (EchoDurationUs < USONIC_ECHO_MAX_US)
  {
    return (uint16_t)(((EchoDurationUs * 17) + (500UL))/1000UL); 
  }
  return 0;
}

uint16_t UltraSonic_Filter(uint16_t New, uint16_t Old)
{
  return (uint16_t)(((uint32_t)New+(3UL*(uint32_t)Old)+2UL)/4UL);
  //return New;
}

void LEDS_Update(uint8_t Percent)
{
  uint8_t LED_Count = 0;
  for(uint8_t i=0; i<LED_COUNT; ++i)
  {
    LEDs[i] = CRGB(0, 0, 0);
  }
  
  if (Percent >   0)  { LEDs[0]  = CRGB(0*LED_R_K,  11*LED_G_K, 0); ++LED_Count; }
  if (Percent >=  9)  { LEDs[1]  = CRGB(1*LED_R_K,  10*LED_G_K, 0); ++LED_Count; }
  if (Percent >= 18)  { LEDs[2]  = CRGB(2*LED_R_K,  9 *LED_G_K, 0); ++LED_Count; }
  if (Percent >= 27)  { LEDs[3]  = CRGB(3*LED_R_K,  8 *LED_G_K, 0); ++LED_Count; }
  if (Percent >= 36)  { LEDs[4]  = CRGB(4*LED_R_K,  7 *LED_G_K, 0); ++LED_Count; }
  if (Percent >= 45)  { LEDs[5]  = CRGB(5*LED_R_K,  6 *LED_G_K, 0); ++LED_Count; }
  if (Percent >= 54)  { LEDs[6]  = CRGB(6*LED_R_K,  5 *LED_G_K, 0); ++LED_Count; }
  if (Percent >= 63)  { LEDs[7]  = CRGB(7*LED_R_K,  4 *LED_G_K, 0); ++LED_Count; }
  if (Percent >= 72)  { LEDs[8]  = CRGB(8*LED_R_K,  3 *LED_G_K, 0); ++LED_Count; }
  if (Percent >= 81)  { LEDs[9]  = CRGB(9*LED_R_K,  2 *LED_G_K, 0); ++LED_Count; }
  if (Percent >= 90)  { LEDs[10] = CRGB(10*LED_R_K, 1 *LED_G_K, 0); ++LED_Count; }
  if (Percent >= 99)  { LEDs[11] = CRGB(11*LED_R_K, 0 *LED_G_K, 0); ++LED_Count; }

  // Apply Mode 
  switch(LED_Mode)
  {
    case LED_MODE__ONE_LED:
    {    
      for(uint8_t i=LED_COUNT-1; i>0; --i)
      {
        if (CRGB(0, 0, 0) != LEDs[i])
        {
          // Cancel previous LEDs
          while (i--)
          {
            LEDs[i] = CRGB(0, 0, 0); 
          }
          break;
        }
      }
    }
    break;
      
    case LED_MODE__RAINBOW:
    { 
      fill_rainbow(LEDs, LED_Count, LED_Hue, 7);
    }
    break;
    
    case LED_MODE__RAINBOW_GLITTER:
    {
      fill_rainbow(LEDs, LED_Count, LED_Hue, 7);
      if( random8() < 80/*chanceOfGlitter*/) {
        LEDs[ random16(LED_Count) ] += CRGB::White;
      }
    }
    break;
      
    case LED_MODE__CONFETTI:
    {
      fadeToBlackBy(LEDs, LED_Count, 10);
      LEDs[random16(LED_Count)] += CHSV( LED_Hue + random8(64), 200, 255);
    }
    break;
    
    case LED_MODE__SINELON:
    {
      // a colored dot sweeping back and forth, with fading trails
      fadeToBlackBy( LEDs, LED_Count, 20);
      int pos = beatsin16(13,0,LED_Count);
      LEDs[pos] += CHSV( LED_Hue, 255, 192);
    }
    break;
    
    case LED_MODE__BPM:
    {
      // colored stripes pulsing at a defined Beats-Per-Minute (BPM)
      uint8_t BeatsPerMinute = 62;
      CRGBPalette16 palette = PartyColors_p;
      uint8_t beat = beatsin8( BeatsPerMinute, 64, 255);
      for(uint8_t i = 0; i < LED_Count; i++) { //9948
        LEDs[i] = ColorFromPalette(palette, LED_Hue+(i*2), beat-LED_Hue+(i*10));
      }
    }
    break;
    
    case LED_MODE__JUGGLE:
    {
      // eight colored dots, weaving in and out of sync with each other
      fadeToBlackBy( LEDs, LED_Count, 20);
      byte dothue = 0;
      for(uint8_t i = 0; i < 8; i++) {
        LEDs[beatsin16(i+7,0,LED_Count)] |= CHSV(dothue, 200, 255);
        dothue += 32;
      }
    }
    break;
  }
 
  // Final Update
  FastLED.show();
}
 
void loop()
{
  // Compute Distance in CM
  Distance_CM = UltraSonic_Filter(UltraSonic_Distance_CM(), Distance_CM);
  
  // Convert Distance in %
  uint8_t Percent = 0;
  if ((Distance_CM > 0) && (Distance_CM < DISTANCE_MAX_CM))
  {
    if (Distance_CM > DISTANCE_MIN_CM)
    {
      Percent = 100 - (uint8_t)((100*(Distance_CM - DISTANCE_MIN_CM)) / (DISTANCE_MAX_CM - DISTANCE_MIN_CM));
    }
    else
    {
      Percent = 100;
    }
  }

  // Print Debug
  // Serial.print("Distance in CM: "); Serial.println(Distance_CM);
  // Serial.print("Distance in %:" );  Serial.println(Percent);
  // Serial.println(" ");
  
  // Wait Loop
  while ((uint32_t)millis() - Loop_Ref_Ms < LOOP_UPDATE_MS)
  {
    // Detect Press
    if (LOW == digitalRead(BP_INPUT))
    {
      // Wait Release
      delay(100);
      Loop_Ref_Ms = millis();
      while(LOW == digitalRead(BP_INPUT));
      Loop_Ref_Ms = (uint32_t)millis() - Loop_Ref_Ms;
      
      // Change Mode (short press)
      if (Loop_Ref_Ms < 500)
      {
        LED_Mode = (LED_Mode_t)(LED_Mode+1);
        if (LED_MODE__COUNT == LED_Mode)
        {
          LED_Mode = (LED_Mode_t)0;
        }      
        Debug_PrintMode();
      }
      // Change brightness (long press)
      else
      {
        switch(LED_Brightness)
        {
          case LED_BRIGHTNESS__LOW: LED_Brightness = LED_BRIGHTNESS__MED; break;
          case LED_BRIGHTNESS__MED: LED_Brightness = LED_BRIGHTNESS__HIG; break;
          default: LED_Brightness = LED_BRIGHTNESS__LOW;
        }
        Debug_PrintBrightness();

        // set master brightness control
        FastLED.setBrightness(LED_Brightness);
      }
    }
    
    // Update LEDs 
    ++LED_Hue;
    LEDS_Update(Percent);

    // Delay
    delay(20);
  }
  ++Loop_Counter;
  Loop_Ref_Ms = millis();
}