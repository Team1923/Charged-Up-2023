#include <FastLED.h>

#define NUM_STRIPS 1
#define NUM_LEDS_PER_STRIP 101
CRGB leds[NUM_LEDS_PER_STRIP];

#define LED_PIN_1 10

#define BRIGHTNESS 175
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

long white_counter = 0;

void setup() {
  pinMode(COM_PIN_1, INPUT);
  pinMode(COM_PIN_2, INPUT);
  pinMode(COM_PIN_3, INPUT);

  //note that this model uses GRB formatting
  FastLED.addLeds<WS2812B, LED_PIN_1, RGB>(leds, NUM_LEDS_PER_STRIP);
  Serial.begin(9600);
}

void loop() {
  handleCurrentSelection();
  lightUp();
  Serial.println(currentState);
}

void setYellow() {
    white_counter = 0;

    for (int j = 0; j < NUM_LEDS_PER_STRIP; j++) {
      leds[j] = CRGB(255, 255, 0);
    }

  FastLED.show();
}

void setPurple() {
  white_counter = 0;

    for (int j = 0; j < NUM_LEDS_PER_STRIP; j++) {
      leds[j] = CRGB(0, 119, 200);
    }
  
  FastLED.show();
}

void setGreen() {
  white_counter = 0;

  for (int j = 0; j < NUM_LEDS_PER_STRIP; j++) {
    leds[j] = CRGB(255, 0, 0);
  }
  FastLED.show();
}

void setWhite(){

  CRGB desired_color;

  white_counter += 2;

  if(white_counter < 300) {
    int div = (white_counter / 20) % 2;
    if(div == 1) {
      desired_color = CRGB(255,255,255);
    } else {
      desired_color = CRGB(0,0,0);
    }
  } else {
    desired_color = CRGB(0, 119, 200);
  }
  
  // if((white_counter > 20 && white_counter < 40) || (white_counter > 60 && white_counter < 80)  || white_counter > 100) {
  //   desired_color = CRGB(255,255,255);
  // } else {
  //   desired_color = CRGB(0,0,0);
  // }

  for(int j = 0; j < NUM_LEDS_PER_STRIP; j++){
    leds[j] = desired_color;
  }
  FastLED.show();
}

void redOscillating() {
  white_counter = 0;
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
    leds[j] = CRGB(0, currentRed, 0);

  }

  FastLED.show();
}

void RainbowOscillating(){
    white_counter = 0;

    if(colorchange){
      currentColor += 2; 
    }
    for(int j = 0; j<NUM_LEDS_PER_STRIP; j++){
      if(currentColor > 255){
        currentColor = 0; 
      }
      leds[j] = CHSV(currentColor - (j * 2), SATURATION, BRIGHTNESS);
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
    case 0:
      redOscillating();
      break;
    case 1:
      setGreen();
      break;
    case 7:
      setWhite();
      break;
    case 4:
      RainbowOscillating();
      break;
    case 3:
      setPurple();
    default:
      break;
  }
}
