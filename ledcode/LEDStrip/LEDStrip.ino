#include <FastLED.h>

#define NUM_STRIPS 2
#define NUM_LEDS_PER_STRIP 100
CRGB leds[NUM_STRIPS][NUM_LEDS_PER_STRIP];

#define LED_PIN_1 10
#define LED_PIN_2 11

#define BRIGHTNESS 255
#define SATURATION 255


//Define digital input pins from RoboRio
#define COM_PIN_1 2 // pin that controls arm 
#define COM_PIN_2 3
#define COM_PIN_3 4

bool getBrighter = true;
int currentRed = 0;

int currentColor = 0; 
bool colorchange = true;


int currentState = 0;


void setup() {
  pinMode(COM_PIN_1, INPUT);
  pinMode(COM_PIN_2, INPUT);
  pinMode(COM_PIN_3, INPUT);

  //note that this model uses GRB formatting
  FastLED.addLeds<WS2812B, LED_PIN_1, RGB>(leds[0], NUM_LEDS_PER_STRIP);
  FastLED.addLeds<WS2812B, LED_PIN_2, RGB>(leds[1], NUM_LEDS_PER_STRIP);
  Serial.begin(9600);
}

void loop() {
  handleCurrentSelection();
  lightUp();
  Serial.println(currentState);
}

void setYellow() {
  for (int i = 0; i < NUM_STRIPS; i++) {
    for (int j = 0; j < NUM_LEDS_PER_STRIP; j++) {
      leds[i][j] = CRGB(255, 255, 0);
    }
  }
  FastLED.show();
}

void setPurple() {
  for (int i = 0; i < NUM_STRIPS; i++) {
    for (int j = 0; j < NUM_LEDS_PER_STRIP; j++) {
      leds[i][j] = CRGB(0, 119, 200);
    }
  }
  FastLED.show();
}

void setGreen() {
  for (int j = 0; j < NUM_LEDS_PER_STRIP; j++) {
    leds[0][j] = CRGB(255, 0, 0);
    leds[1][j] = CRGB(255,0,0);
  }
  FastLED.show();
}

void setWhite(){
  for(int j = 0; j < NUM_LEDS_PER_STRIP; j++){
    leds[0][j] = CRGB(255,255,255);
    leds[1][j] = CRGB(255,255,255);
  }
  FastLED.show();
}

void redOscillating() {
  if (getBrighter) {
    currentRed += 5;
  } else {
    currentRed -= 5;
  }

  for (int j = 0; j < NUM_LEDS_PER_STRIP; j++) {
    if (currentRed >= 255) {
      currentRed = 255;
      getBrighter = false;
    } else if (currentRed <= 0) {
      currentRed = 0;
      getBrighter = true;
    }
    leds[0][j] = CRGB(0, currentRed, 0);
    leds[1][j] = CRGB(0, currentRed, 0);
  }

  FastLED.show();
}

void RainbowOscillating(){
    if(colorchange){
      currentColor += 5; 
    }
    for(int j = 0; j<NUM_LEDS_PER_STRIP; j++){
      if(currentColor > 255){
        currentColor = 0; 
      }
      leds[0][j] = CHSV(currentColor - (j * 2), SATURATION, BRIGHTNESS);
      leds[1][j] = CHSV(currentColor - (j * 2), SATURATION, BRIGHTNESS);
    }
		FastLED.show();  
    serial.ln(currentColor);
}

void handleCurrentSelection() {
  for (int i = 2; i <= 4; i++) {
    bitWrite(currentState, i-2, digitalRead(i));
  }
}

void lightUp() {

  switch (currentState) {
    case 0:
      redOscillating();
      break;
    case 4:
      setGreen();
      break;
    case 6:
      setPurple();
      break;
    case 7:
      setWhite();
      break;
    case 1:
      RainbowOscillating();
      break;
    default:
      break;
  }
}
