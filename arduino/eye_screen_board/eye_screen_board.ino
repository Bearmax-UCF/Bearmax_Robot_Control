#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <SPI.h>
#include "draw_utils.h"

//#include "./eyes_closed.h"

// Only define on the left eye!!!
#define LEFT_EYE

#define BAUD_RATE 9600

#define TFT_WIDTH  128
#define TFT_HEIGHT 128

// Color definitions
#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0  
#define WHITE   0xFFFF
#define BROWN   0x6227
//60463B
// 96, 70, 59

#define EYELID_COLOR WHITE

#define TFT_CS 10
#define TFT_RST 9
#define TFT_DC 8

Bearmax_TFT tft = Bearmax_TFT(TFT_CS, TFT_DC, TFT_RST);

enum Mode {
  NORMAL,
  HAPPY,
  ANGRY,
  SAD,
  DRUNK,
  CLOSED
};

Mode current_mode = NORMAL;

void handle_cmd() {
  if (Serial.available() > 0) {
    String cmd = Serial.readString();
    cmd.trim();
    if (cmd.startsWith("m ")) {
      String mode_id = cmd.substring(2);
      if (mode_id == "normal") {
        current_mode = NORMAL;
      } else if (mode_id == "happy") {
        current_mode = HAPPY;
      } else if (mode_id == "angry") {
        current_mode = ANGRY;
      } else if (mode_id == "sad") {
        current_mode = SAD;
      } else if (mode_id == "drunk") {
        current_mode = DRUNK;
      } else if (mode_id == "closed") {
        current_mode = CLOSED;
      } else {
        Serial.println("[Error]: Invalid Mode");
      }
    } else {
      Serial.println("[Error]: Invalid Command");
    }
  }
}

void setup() {
  Serial.begin(BAUD_RATE);

  tft.initR(INITR_144GREENTAB);

  tft.fillScreen(BLACK);

  //tft.drawRGBBitmap(0, 0, EYES_CLOSED, tft.width(), tft.height());
  //tft.drawLine(0, 0, 128, 128, RED);

  //tft.fillAngledRect(TFT_WIDTH/2, TFT_HEIGHT/2, 50, 30, 45, WHITE);
}

long previousMillis = 0;
long blinkInterval = 10000;
bool doBlink = false;

#define FRAMES 16
#define FPS 10

void blink() {
  // CLOSE EYE
  for (int i=1; i<=FRAMES; i++) {
   tft.fillRect(4*i, 0, 4, TFT_HEIGHT, EYELID_COLOR);
   tft.fillRect(TFT_WIDTH - (4*i), 0, 4, TFT_HEIGHT, EYELID_COLOR);
   delay(FPS); 
  }
  delay(250); // How long to hold eyelids shut
  // OPEN EYE
  for (int i=FRAMES; i>=1; i--) {
   tft.fillRect(4*i, 0, 4, TFT_HEIGHT, BLACK);
   tft.fillRect(TFT_WIDTH - (4*i), 0, 4, TFT_HEIGHT, BLACK);
   delay(FPS); 
  }
}

void happy() {
  // Bottom Eyelid
  #ifdef LEFT_EYE
  tft.fillRect(0, 0, TFT_WIDTH / 2, TFT_HEIGHT, EYELID_COLOR);
  #else
  tft.fillRect(TFT_WIDTH - 60, 0, TFT_WIDTH / 2, TFT_HEIGHT, EYELID_COLOR);
  #endif
}

void loop() {/*
  handle_cmd();

  if (millis() - previousMillis >= blinkInterval) {
    previousMillis += blinkInterval;

    // TODO: Maybe add randomness to blink interval?
    doBlink = !doBlink;
  }

  // Execute Animation Mode
  switch (current_mode) {
    case NORMAL:
      if (doBlink) {
       blink();
       // RESET EYE
       tft.fillScreen(BLACK);
       doBlink = false; 
      }
      break;
    case HAPPY:
      happy();
    default:
      break;
  }*/
}

void fillAngledRect(int16_t x, int16_t y, int16_t deg, int16_t w, int16_t h, uint16_t color) {
    if (!w || !h) { return; }

    // TODO: Clip?

    tft.startWrite();

    // TODO: draw vlines bitch

    tft.endWrite();
}
