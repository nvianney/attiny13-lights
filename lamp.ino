#define LIGHT_PIN 2
#define LIGHT_COUNT 10
// 0.125 us +- 0.15
// 0.25
// 0.375
// 0.5
// 0.625
// 0.75
// 0.875

// T0H 0.4us   (0.225, 0.55)
// T0L 0.85us  (0.7, 1)

// T1H 0.8us
// T1L 0.45us

struct Lights {
    uint8_t green;
    uint8_t red;
    uint8_t blue;
} lights;

void setupLights(struct Lights &lights) {
    lights.green = 0xFF;
    lights.red = 0xFF;
    lights.blue = 0xFF;

    pinMode(LIGHT_PIN, OUTPUT);
}

void writeLights(struct Lights &lights) {
    const uint8_t mask = digitalPinToBitMask(LIGHT_PIN);
    volatile uint8_t hi = PORTB | mask;
    volatile uint8_t lo = PORTB & (~mask);
    const uint32_t data = ((uint32_t) lights.green << 16) | ((uint32_t) lights.red << 8) | ((uint32_t) lights.blue);
    volatile uint8_t *ptr = ((uint8_t*) &data) + 1;
    volatile uint8_t byte = *(ptr++);
    volatile uint8_t temp = (data & 0x08) ? hi : lo;

    // max 3 instructions per write

    asm volatile(
        "top:\n\t"
        "out %[port], %[hi]\n\t"
        "nop \n\t"
        "nop \n\t"
        ""

        : [temp] "+r" (temp)
        : [port] "I" (_SFR_IO_ADDR(PORTB)),
          [hi] "r" (hi),
          [lo] "r" (lo),
          [byte] "r" (byte),
          [start] "e" (ptr),
          [ptr] "e" (ptr)
    );
}

void setup() {
    setupLights(lights);
}

void loop() {

}
