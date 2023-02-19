#include <FastLED.h>

#define NUM_STRIPS 2
#define NUM_LEDS_PER_STRIP 300
CRGB leds[NUM_STRIPS][NUM_LEDS_PER_STRIP];

#define LED_PIN_1 3
#define LED_PIN_2 5

//Define digital input pins from RoboRio
#define COM_PIN_1 0
#define COM_PIN_2 1
#define COM_PIN_3 2

bool getBrighter = true;
int currentRed = 0;

int currentState = 0;

void setup() {
  pinMode(COM_PIN_1, INPUT);
  pinMode(COM_PIN_2, INPUT);
  pinMode(COM_PIN_3, INPUT);

  //note that this model uses GRB formatting
  FastLED.addLeds<WS2812B, LED_PIN_1, RGB>(leds[0], NUM_LEDS_PER_STRIP);
  FastLED.addLeds<WS2812B, LED_PIN_2, RGB>(leds[1], NUM_LEDS_PER_STRIP);
}

void loop() {
  handleCurrentSelection();
  lightUp();
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

void setGreen(int desiredLEDStrip) {
  for (int j = 0; j < NUM_LEDS_PER_STRIP; j++) {
    leds[desiredLEDStrip][j] = CRGB(255, 0, 0);
  }
  FastLED.show();
}

void redOscillating(int desiredLEDStrip) {
  if (getBrighter) {
    currentRed += 15;
  } else {
    currentRed -= 15;
  }

  for (int j = 0; j < NUM_LEDS_PER_STRIP; j++) {
    if (currentRed >= 255) {
      currentRed = 255;
      getBrighter = false;
    } else if (currentRed <= 0) {
      currentRed = 0;
      getBrighter = true;
    }
    leds[desiredLEDStrip][j] = CRGB(0, currentRed, 0);
  }

  FastLED.show();
}

void handleCurrentSelection() {
  for (int i = 0; i <= 2; i++) {
    bitWrite(currentState, i, digitalRead(i));
  }
}

void lightUp() {
  //led 0 = arm
  //led 1 = intake
  //000
  switch (currentState) {
    case 0:
      redOscillating(0);
      redOscillating(1);
      break;
    case 1:
      setGreen(0);
      redOscillating(1);
      break;
    case 2:
      redOscillating(0);
      setGreen(1);
      break;
    case 3:
      setGreen(0);
      setGreen(1);
      break;
    case 4:
      setPurple();
      break;
    case 5:
      setYellow();
      break;
    default:
      redOscillating(0);
      redOscillating(1);
  }
}
