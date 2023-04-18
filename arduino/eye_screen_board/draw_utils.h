#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

class Bearmax_TFT : public Adafruit_ST7735 {
  public:
    Bearmax_TFT(int8_t cs, int8_t dc, int8_t rst);
    virtual void fillAngledRect(int16_t x, int16_t y, int16_t width, int16_t height, int16_t angle, uint16_t color);
};
