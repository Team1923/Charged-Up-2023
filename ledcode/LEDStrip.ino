#include <FastLED.h>

#define NUM_STRIPS 2
#define NUM_LEDS_PER_STRIP 300
CRGB leds[NUM_STRIPS][NUM_LEDS_PER_STRIP];

#define LED_PIN_1 1
#define LED_PIN_2 2

//Define digital input pins from RoboRio
#define COM_PIN_1 5
#define COM_PIN_2 6

bool getBrighter = true;
int currentRed = 0;

void setup() {
  pinMode(COM_PIN_1, INPUT);
  pinMode(COM_PIN_2, INPUT);

  //note that this model uses GRB formatting
  FastLED.addLeds<WS2812B, LED_PIN_1, RGB>(leds[0], NUM_LEDS_PER_STRIP);
  FastLED.addLeds<WS2812B, LED_PIN_2, RGB>(leds[1], NUM_LEDS_PER_STRIP);
}

void loop() {
  handleCurrentSelection();
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
  for (int i = 0; i < NUM_STRIPS; i++) {
    for (int j = 0; j < NUM_LEDS_PER_STRIP; j++) {
      leds[i][j] = CRGB(255, 0, 0);
    }
  }
  FastLED.show();
}

void redOscillating() {
  if (getBrighter) {
    currentRed += 15;
  } else {
    currentRed -= 15;
  }

  for (int i = 0; i < NUM_STRIPS; i++) {
    for (int j = 0; j < NUM_LEDS_PER_STRIP; j++) {
      if (currentRed >= 255) {
        currentRed = 255;
        getBrighter = false;
      } else if (currentRed <= 0) {
        currentRed = 0;
        getBrighter = true;
      }
    }
  }

  FastLED.show();
}

void handleCurrentSelection() {
  //COM_PIN_1, COM_PIN_2
  if (digitalRead(COM_PIN_1) == false && digitalRead(COM_PIN_2) == false) {
    redOscillating();
  }

  if (digitalRead(COM_PIN_1) == true && digitalRead(COM_PIN_2) == false) {
    setGreen();
  }

  if (digitalRead(COM_PIN_1) == false && digitalRead(COM_PIN_2) == true) {
    setPurple();
  }

  if (digitalRead(COM_PIN_1) == true && digitalRead(COM_PIN_2) == true) {
    setYellow();
  }
}
