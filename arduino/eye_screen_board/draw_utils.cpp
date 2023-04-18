#include "draw_utils.h"

Bearmax_TFT::Bearmax_TFT(int8_t cs, int8_t dc, int8_t rst)
  : Adafruit_ST7735(cs, dc, rst) {}


void Bearmax_TFT::fillAngledRect(int16_t x, int16_t y, int16_t width, int16_t height, int16_t angle, uint16_t color) {
  // Convert angle to radians
  float radAngle = angle * PI / 180.0;

  // Calculate half the width and height
  float halfWidth = width / 2.0;
  float halfHeight = height / 2.0;

  // Calculate the coordinates of the four corners of the rectangle
  int16_t x1 = x - halfWidth * cos(radAngle) + halfHeight * sin(radAngle); // top left x coordinate
  int16_t y1 = y - halfWidth * sin(radAngle) - halfHeight * cos(radAngle); // top left y coordinate
  int16_t x2 = x + halfWidth * cos(radAngle) + halfHeight * sin(radAngle); // top right x coordinate
  int16_t y2 = y - halfWidth * sin(radAngle) + halfHeight * cos(radAngle); // top right y coordinate
  int16_t x3 = x + halfWidth * cos(radAngle) - halfHeight * sin(radAngle); // bottom right x coordinate
  int16_t y3 = y + halfWidth * sin(radAngle) + halfHeight * cos(radAngle); // bottom right y coordinate
  int16_t x4 = x - halfWidth * cos(radAngle) - halfHeight * sin(radAngle); // bottom left x coordinate
  int16_t y4 = y + halfWidth * sin(radAngle) - halfHeight * cos(radAngle); // bottom left y coordinate

  this->fillTriangle(x1, y1, x2, y2, x3, y3, color); // Top Triangle
  this->fillTriangle(x1, y1, x3, y3, x4, y4, color); // Bottom Triangle
}
