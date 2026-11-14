#pragma once
#include <stdint.h>
#include <math.h>

#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define LOW 0
#define HIGH 1
#define CHANGE 2
#define constrain(x, a, b) ((x) < (a) ? (a) : ((x) > (b) ? (b) : (x)))
extern uint32_t mockMicros;
extern int mockDigital[256];
inline unsigned long micros() { return mockMicros; }
inline unsigned long millis() { return mockMicros / 1000; }
inline void delayMicroseconds(unsigned int us) { mockMicros += us; }
inline void pinMode(uint8_t, int) {}
inline void digitalWrite(uint8_t pin, int value) { mockDigital[pin] = value; }
inline int digitalRead(uint8_t pin) { return mockDigital[pin]; }
inline void analogWrite(uint8_t, int) {}
inline int digitalPinToInterrupt(uint8_t pin) { return pin; }
inline void attachInterrupt(int, void (*)(), int) {}
struct MockSerial {
    void begin(unsigned long) {}
    int available() const { return 0; }
    int availableForWrite() const { return 64; }
    int read() { return -1; }
    template <class T> void println(const T&) {}
};
extern MockSerial Serial;
