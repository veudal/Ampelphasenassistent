#include <Arduino.h>
#include <Adafruit_GFX.h>

class UTFT : public Adafruit_GFX {
public:
    static constexpr int16_t WIDTH = 320;
    static constexpr int16_t HEIGHT = 240;

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

    UTFT() : Adafruit_GFX(WIDTH, HEIGHT) {}

    void begin() {
        // Enable clocks
        RCC->APB2ENR |= RCC_APB2ENR_IOPCEN |
                        RCC_APB2ENR_IOPDEN |
                        RCC_APB2ENR_IOPEEN |
                        RCC_APB2ENR_AFIOEN;

        RCC->AHBENR |= RCC_AHBENR_FSMCEN;

        // Configure FSMC pins
        const int dPins[] = {0, 1, 4, 5, 7, 8, 9, 10, 11, 14, 15};

        for (unsigned int i = 0; i < sizeof(dPins) / sizeof(dPins[0]); i++)
            configurePin(GPIOD, dPins[i]);

        for (int pin = 7; pin <= 15; pin++)
            configurePin(GPIOE, pin);

        // FSMC timing/configuration
        FSMC_Bank1->BTCR[1] =
            1 | (15u << 4) | (5u << 8) | (15u << 20) | (15u << 24);

        FSMC_Bank1->BTCR[0] = 0x1011;

        // LCD reset
        pinMode(PC1, OUTPUT);
        pinMode(PE1, OUTPUT);

        digitalWrite(PC1, LOW);
        digitalWrite(PE1, LOW);
        delay(50);

        digitalWrite(PC1, HIGH);
        digitalWrite(PE1, HIGH);
        delay(120);

        // ILI9325 initialization
        static const uint16_t init[][2] = {
            {0x00, 0x0001}, {0x01, 0x0100},
            {0x02, 0x0700}, {0x03, 0x1038},
            {0x04, 0x0000}, {0x08, 0x0202},
            {0x09, 0x0000}, {0x0A, 0x0000},
            {0x0C, 0x0001}, {0x0D, 0x0000},
            {0x0F, 0x0000}, {0x10, 0x0000},
            {0x11, 0x0007}, {0x12, 0x0000},
            {0x13, 0x0000}, {0xFFFF, 200},
            {0x10, 0x1590}, {0x11, 0x0227},
            {0xFFFF, 50}, {0x12, 0x009C},
            {0xFFFF, 50}, {0x13, 0x1900},
            {0x29, 0x1900}, {0x2B, 0x000E},
            {0xFFFF, 50}, {0x60, 0x2700},
            {0x61, 0x0001}, {0x6A, 0x0000},
            {0x07, 0x0133}, {0xFFFF, 50}
        };

        for (unsigned int i = 0; i < sizeof(init) / sizeof(init[0]); i++) {
            if (init[i][0] == 0xFFFF)
                delay(init[i][1]);
            else
                writeRegister(init[i][0], init[i][1]);
        }

        setTextSize(1);
        setTextWrap(true);
        setTextColor(WHITE, BLACK);
        fillScreen(BLACK);
    }

    // UTFT-style color selection
    void setColor(uint8_t r, uint8_t g, uint8_t b) {
        foreground = color565(r, g, b);
        setTextColor(foreground, background);
    }

    // Adafruit GFX drawing backend
    void drawPixel(int16_t x, int16_t y, uint16_t color) override {
        writeFillRect(x, y, 1, 1, color);
    }

    void writeFillRect(int16_t x, int16_t y,
                       int16_t w, int16_t h,
                       uint16_t color) override {

        if (x < 0) { w += x; x = 0; }
        if (y < 0) { h += y; y = 0; }

        if (x + w > WIDTH)  w = WIDTH - x;
        if (y + h > HEIGHT) h = HEIGHT - y;

        if (w <= 0 || h <= 0) return;

        // 180-degree rotation
        x = WIDTH - (x + w);
        y = HEIGHT - (y + h);

        writeRegister(0x50, y);
        writeRegister(0x51, y + h - 1);
        writeRegister(0x52, x);
        writeRegister(0x53, x + w - 1);

        writeRegister(0x20, y);
        writeRegister(0x21, x);

        LCD_REGISTER = 0x0022;

        for (uint32_t i = 0; i < (uint32_t)w * h; i++)
            LCD_DATA = color;
    }

    static uint16_t color565(uint8_t r, uint8_t g, uint8_t b) {
        return ((uint16_t)(r & 0xF8) << 8) |
               ((uint16_t)(g & 0xFC) << 3) |
               (b >> 3);
    }

private:
    uint16_t foreground = WHITE;
    uint16_t background = BLACK;

    volatile uint16_t &LCD_REGISTER =
        *reinterpret_cast<volatile uint16_t *>(0x60000000);

    volatile uint16_t &LCD_DATA =
        *reinterpret_cast<volatile uint16_t *>(0x60020000);

    static void configurePin(GPIO_TypeDef *port, int pin) {
        volatile uint32_t *reg = (pin < 8) ? &port->CRL : &port->CRH;
        int shift = (pin % 8) * 4;

        *reg = (*reg & ~(0xFu << shift)) | (0xBu << shift);
    }

    void writeRegister(uint16_t reg, uint16_t value) {
        LCD_REGISTER = reg;
        LCD_DATA = value;
    }
};
