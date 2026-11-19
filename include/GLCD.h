#pragma once

#include <Arduino.h>
#include <Adafruit_GFX.h>

#ifndef LCD_ROTATE_180
#define LCD_ROTATE_180 0
#endif

// 320x240 FSMC display using the same controller/register sequence
// as the supplied Lcd.cpp implementation.
class GLCD : public Adafruit_GFX {
public:
    static constexpr int16_t WIDTH  = 320;
    static constexpr int16_t HEIGHT = 240;

    // RGB565 colors
    static constexpr uint16_t BLACK   = 0x0000;
    static constexpr uint16_t WHITE   = 0xFFFF;
    static constexpr uint16_t RED     = 0xF800;
    static constexpr uint16_t GREEN   = 0x07E0;
    static constexpr uint16_t BLUE    = 0x001F;
    static constexpr uint16_t YELLOW  = 0xFFE0;
    static constexpr uint16_t CYAN    = 0x07FF;
    static constexpr uint16_t MAGENTA = 0xF81F;
    static constexpr uint16_t GRAY    = 0x8410;
    static constexpr uint16_t DKGRAY  = 0x4208;

    GLCD() : Adafruit_GFX(WIDTH, HEIGHT) {}

    void begin() {
        // Configure FSMC GPIO pins.
        RCC->APB2ENR |= RCC_APB2ENR_IOPCEN |
                        RCC_APB2ENR_IOPDEN |
                        RCC_APB2ENR_IOPEEN |
                        RCC_APB2ENR_AFIOEN;
        RCC->AHBENR |= RCC_AHBENR_FSMCEN;

        const int dPins[] = {0, 1, 4, 5, 7, 8, 9, 10, 11, 14, 15};
        for (unsigned int i = 0; i < sizeof(dPins) / sizeof(dPins[0]); ++i)
            configurePin(GPIOD, dPins[i]);

        for (int pin = 7; pin <= 15; ++pin)
            configurePin(GPIOE, pin);

        FSMC_Bank1->BTCR[1] =
            1 | (15u << 4) | (5u << 8) | (15u << 20) | (15u << 24);
        FSMC_Bank1->BTCR[0] = 0x1011;

        // LCD reset pins, matching the supplied working Lcd.cpp.
        pinMode(PC1, OUTPUT);
        pinMode(PE1, OUTPUT);
        digitalWrite(PC1, LOW);
        digitalWrite(PE1, LOW);
        delay(50);
        digitalWrite(PC1, HIGH);
        digitalWrite(PE1, HIGH);
        delay(120);

        // Controller initialization sequence from the supplied code.
        static const uint16_t setup[][2] = {
            {0x00, 0x0001}, {0x01, 0x0100}, {0x02, 0x0700},
            {0x03, 0x1038}, {0x04, 0x0000}, {0x08, 0x0202},
            {0x09, 0x0000}, {0x0A, 0x0000}, {0x0C, 0x0001},
            {0x0D, 0x0000}, {0x0F, 0x0000}, {0x10, 0x0000},
            {0x11, 0x0007}, {0x12, 0x0000}, {0x13, 0x0000},
            {0xFFFF, 200},
            {0x10, 0x1590}, {0x11, 0x0227},
            {0xFFFF, 50},
            {0x12, 0x009C},
            {0xFFFF, 50},
            {0x13, 0x1900}, {0x29, 0x1900}, {0x2B, 0x000E},
            {0xFFFF, 50},
            {0x60, 0x2700}, {0x61, 0x0001}, {0x6A, 0x0000},
            {0x07, 0x0133},
            {0xFFFF, 50}
        };

        for (unsigned int i = 0; i < sizeof(setup) / sizeof(setup[0]); ++i) {
            if (setup[i][0] == 0xFFFF)
                delay(setup[i][1]);
            else
                writeRegister(setup[i][0], setup[i][1]);
        }

        setTextSize(1);
        setTextWrap(true);
        setTextColor(foreground, background);
        fillScreen(BLACK);
    }

    // Adafruit_GFX drawing backend.
    void drawPixel(int16_t x, int16_t y, uint16_t color) override {
        writeFillRect(x, y, 1, 1, color);
    }

    void writeFillRect(int16_t x, int16_t y, int16_t w, int16_t h,
                       uint16_t color) override {
        if (x < 0) { w += x; x = 0; }
        if (y < 0) { h += y; y = 0; }
        if (x + w > WIDTH)  w = WIDTH - x;
        if (y + h > HEIGHT) h = HEIGHT - y;
        if (w <= 0 || h <= 0) return;

#if LCD_ROTATE_180
        x = WIDTH - (x + w);
        y = HEIGHT - (y + h);
#endif

        setWindow(x, y, w, h);
        for (uint32_t i = 0; i < (uint32_t)w * (uint32_t)h; ++i)
            LCD_DATA = color;
    }

    // UTFT-like color selection. Convenience drawing overloads below use it.
    void setColor(uint8_t r, uint8_t g, uint8_t b) {
        foreground = color565(r, g, b);
        setTextColor(foreground, background);
    }

    void setBackColor(uint8_t r, uint8_t g, uint8_t b) {
        background = color565(r, g, b);
        setTextColor(foreground, background);
    }

    void clrScr() {
        fillScreen(BLACK);
    }

    void fillScr(uint8_t r, uint8_t g, uint8_t b) {
        fillScreen(color565(r, g, b));
    }

    // Keep Adafruit_GFX overloads that accept an explicit color available too.
    using Adafruit_GFX::drawPixel;
    using Adafruit_GFX::drawLine;
    using Adafruit_GFX::drawRect;
    using Adafruit_GFX::fillRect;
    using Adafruit_GFX::drawCircle;
    using Adafruit_GFX::fillCircle;
    using Adafruit_GFX::drawRoundRect;
    using Adafruit_GFX::fillRoundRect;

    // Convenience methods that use the current setColor().
    void drawPixel(int16_t x, int16_t y) {
        drawPixel(x, y, foreground);
    }

    void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1) {
        Adafruit_GFX::drawLine(x0, y0, x1, y1, foreground);
    }

    void drawRect(int16_t x, int16_t y, int16_t w, int16_t h) {
        Adafruit_GFX::drawRect(x, y, w, h, foreground);
    }

    void fillRect(int16_t x, int16_t y, int16_t w, int16_t h) {
        writeFillRect(x, y, w, h, foreground);
    }

    void drawCircle(int16_t x, int16_t y, int16_t r) {
        Adafruit_GFX::drawCircle(x, y, r, foreground);
    }

    void fillCircle(int16_t x, int16_t y, int16_t r) {
        Adafruit_GFX::fillCircle(x, y, r, foreground);
    }

    void drawRoundRect(int16_t x, int16_t y, int16_t w, int16_t h,
                       int16_t r) {
        Adafruit_GFX::drawRoundRect(x, y, w, h, r, foreground);
    }

    void fillRoundRect(int16_t x, int16_t y, int16_t w, int16_t h,
                       int16_t r) {
        Adafruit_GFX::fillRoundRect(x, y, w, h, r, foreground);
    }

    void printAt(const char *text, int16_t x, int16_t y) {
        setCursor(x, y);
        print(text);
    }

    static uint16_t color565(uint8_t r, uint8_t g, uint8_t b) {
        return ((uint16_t)(r & 0xF8) << 8) |
               ((uint16_t)(g & 0xFC) << 3) |
               ((uint16_t)b >> 3);
    }

private:
    uint16_t foreground = WHITE;
    uint16_t background = BLACK;

    // FSMC Bank 1, NE1 command address and NE1/A16 data address.
    volatile uint16_t &LCD_REGISTER =
        *reinterpret_cast<volatile uint16_t *>(0x60000000);
    volatile uint16_t &LCD_DATA =
        *reinterpret_cast<volatile uint16_t *>(0x60020000);

    static void configurePin(GPIO_TypeDef *port, int pin) {
        volatile uint32_t *reg = (pin < 8) ? &port->CRL : &port->CRH;
        const int shift = (pin % 8) * 4;
        *reg = (*reg & ~(0xFu << shift)) | (0xBu << shift);
    }

    void writeRegister(uint16_t reg, uint16_t value) {
        LCD_REGISTER = reg;
        LCD_DATA = value;
    }

    void setWindow(int16_t x, int16_t y, int16_t w, int16_t h) {
        // Keep the exact coordinate/register mapping from the supplied Lcd.cpp.
        writeRegister(0x50, y);
        writeRegister(0x51, y + h - 1);
        writeRegister(0x52, x);
        writeRegister(0x53, x + w - 1);
        writeRegister(0x20, y);
        writeRegister(0x21, x);
        LCD_REGISTER = 0x0022;
    }
};
