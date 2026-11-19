#include <Arduino.h>

class Ultrasonic {
public:
    Ultrasonic(int trigPin, int echoPin)
        : trigPin_(trigPin), echoPin_(echoPin) {}

    void begin() {
        pinMode(trigPin_, OUTPUT);
        pinMode(echoPin_, INPUT);
        digitalWrite(trigPin_, LOW);
    }

    // Returns echo pulse duration in µs, or -1 on timeout
    float measurePulse() {
        sendTrigger();

        if (!waitForEcho(HIGH)) return -1.0f;   // echo start
        int start = micros();

        if (!waitForEcho(LOW)) return -1.0f;    // echo end
        return micros() - start;
    }

private:
    int TIMEOUT_US = 30000;

    int trigPin_;
    int echoPin_;

    void sendTrigger() {
        digitalWrite(trigPin_, LOW);
        delayMicroseconds(2);
        digitalWrite(trigPin_, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigPin_, LOW);
    }

    bool waitForEcho(uint8_t level) {
        int start = micros();
        while (digitalRead(echoPin_) != level) {
            if (micros() - start > TIMEOUT_US) return false;
        }
        return true;
    }
};