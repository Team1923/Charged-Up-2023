#include <FastLED.h>

#define NUM_STRIPS 2
#define NUM_LEDS_PER_STRIP 100
CRGB leds[NUM_STRIPS][NUM_LEDS_PER_STRIP];

#define LED_PIN_1 10
#define LED_PIN_2 11

//Define digital input pins from RoboRio
#define COM_PIN_1 2 // pin that controls arm 
#define COM_PIN_2 3
#define COM_PIN_3 4

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

void setGreen(int desiredLEDStrip) {
  for (int j = 0; j < NUM_LEDS_PER_STRIP; j++) {
    leds[desiredLEDStrip][j] = CRGB(255, 0, 0);
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

void handleCurrentSelection() {
  for (int i = 2; i <= 4; i++) {
    bitWrite(currentState, i-2, digitalRead(i));
  }
}

void lightUp() {

  switch (currentState) {
    if
  }
}
