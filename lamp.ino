#define LIGHT_PIN 2
#define LIGHT_COUNT 4
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
    lights.red = 0x00;
    lights.blue = 0x0F;

    pinMode(LIGHT_PIN, OUTPUT);
}

void writeLights(struct Lights &lights) {
    const uint8_t mask = digitalPinToBitMask(LIGHT_PIN);
    volatile uint8_t hi = PORTB | mask;
    volatile uint8_t lo = PORTB & (~mask);
    const uint32_t data = ((uint32_t) lights.blue << 16) | ((uint32_t) lights.red << 8) | ((uint32_t) lights.green); // AVR is little-endian
    volatile uint8_t *start = ((uint8_t*) &data);
    volatile uint8_t byte = *start;
    volatile uint8_t temp = (data & 0x80) ? hi : lo;
    volatile uint8_t light = 0b0111;
    volatile uint8_t count = LIGHT_COUNT;
    volatile uint8_t zero = 0;


    asm volatile(
        "top: \n\t"
        
        "out %[port], %[hi] \n\t"   // 1    port = HI
        "rjmp .+0 \n\t"             //      nop nop
        "out %[port], %[temp] \n\t" // 1    port = TEMP
        "mov %[temp], %[lo] \n\t"   // 1    TEMP = LO
        "sbrc %[byte], 6 \n\t"      // 1/2  if byte & 0x40
        "mov %[temp], %[hi] \n\t"   // 1    TEMP = HI
        "nop \n\t"
        "out %[port], %[lo] \n\t"
        "rjmp .+0 \n\t"
        "nop \n\t"

        "out %[port], %[hi] \n\t"   // 1    port = HI
        "rjmp .+0 \n\t"             //      nop nop
        "out %[port], %[temp] \n\t" // 1    port = TEMP
        "mov %[temp], %[lo] \n\t"   // 1    TEMP = LO
        "sbrc %[byte], 5 \n\t"      // 1/2  if byte & 0x40
        "mov %[temp], %[hi] \n\t"   // 1    TEMP = HI
        "nop \n\t"
        "out %[port], %[lo] \n\t"
        "rjmp .+0 \n\t"
        "nop \n\t"

        "out %[port], %[hi] \n\t"   // 1    port = HI
        "rjmp .+0 \n\t"             //      nop nop
        "out %[port], %[temp] \n\t" // 1    port = TEMP
        "mov %[temp], %[lo] \n\t"   // 1    TEMP = LO
        "sbrc %[byte], 4 \n\t"      // 1/2  if byte & 0x40
        "mov %[temp], %[hi] \n\t"   // 1    TEMP = HI
        "nop \n\t"
        "out %[port], %[lo] \n\t"
        "rjmp .+0 \n\t"
        "nop \n\t"

        "out %[port], %[hi] \n\t"   // 1    port = HI
        "rjmp .+0 \n\t"             //      nop nop
        "out %[port], %[temp] \n\t" // 1    port = TEMP
        "mov %[temp], %[lo] \n\t"   // 1    TEMP = LO
        "sbrc %[byte], 3 \n\t"      // 1/2  if byte & 0x40
        "mov %[temp], %[hi] \n\t"   // 1    TEMP = HI
        "nop \n\t"
        "out %[port], %[lo] \n\t"
        "rjmp .+0 \n\t"
        "nop \n\t"

        "out %[port], %[hi] \n\t"   // 1    port = HI
        "rjmp .+0 \n\t"             //      nop nop
        "out %[port], %[temp] \n\t" // 1    port = TEMP
        "mov %[temp], %[lo] \n\t"   // 1    TEMP = LO
        "sbrc %[byte], 2 \n\t"      // 1/2  if byte & 0x40
        "mov %[temp], %[hi] \n\t"   // 1    TEMP = HI
        "nop \n\t"
        "out %[port], %[lo] \n\t"
        "rjmp .+0 \n\t"
        "nop \n\t"

        "out %[port], %[hi] \n\t"   // 1    port = HI
        "rjmp .+0 \n\t"             // 2    nop nop
        "out %[port], %[temp] \n\t" // 1    port = TEMP
        "mov %[temp], %[lo] \n\t"   // 1    TEMP = LO
        "sbrc %[byte], 1 \n\t"      // 1/2  if byte & 0x40
        "mov %[temp], %[hi] \n\t"   // 1      TEMP = HI
        "nop \n\t"                  // 1    nop
        "out %[port], %[lo] \n\t"   // 1    port = LO
        "rjmp .+0 \n\t"             // 2    nop nop
        "lsr %[light] \n\t"         // 1    light >> 1

        "out %[port], %[hi] \n\t"   // 1    port = HI
        "sbrs %[light], 0 \n\t"     // 1/2  if light == 0
        "subi %[count], 1 \n\t"     // 1      count--
        "out %[port], %[temp] \n\t" // 1    port = TEMP
        "sbrs %[light], 0 \n\t"     // 1/2  if light == 0 (not literally, but equivalent for our scenario)
        "mov %[ptr], %[start] \n\t" // 1      ptr = start
        "sbrs %[light], 0 \n\t"     // 1/2  if light == 0
        "ldi %[light], 7 \n\t"      // 1      light = 7 (0b111)
        "out %[port], %[lo] \n\t"   // 1    port = LO
        "mov %[temp], %[lo] \n\t"   // 1    temp = LO
        "sbrc %[byte], 0 \n\t"      // 1/2  if byte & 0x01
        "mov %[temp], %[hi] \n\t"   // 1      temp = HI

        "out %[port], %[hi] \n\t"   // 1    port = HI
        "ld %[byte], %a[ptr]+ \n\t" // 2    byte = *ptr++
        "out %[port], %[temp] \n\t" // 1    port = TEMP
        "nop \n\t"                  // 1    nop
        "mov %[temp], %[lo] \n\t"   // 1    temp = LO
        "sbrc %[byte], 7 \n\t"      // 1/2  if byte & 0x80
        "mov %[temp], %[hi] \n\t"   // 1      temp = HI
        "out %[port], %[lo] \n\t"   // 1    port = LO
        "cpse %[count], %[zero] \n\t"     // 1/2  if count != 0
        "rjmp top \n\t"             // 2    goto top

        : [temp] "+r" (temp),
          [light] "+r" (light),
          [count] "+r" (count)
        : [port] "I" (_SFR_IO_ADDR(PORTB)),
          [hi] "r" (hi),
          [lo] "r" (lo),
          [byte] "r" (byte),
          [zero] "r" (zero),
          [start] "e" (start),
          [ptr] "e" (start+1)
    );
    digitalWrite(LIGHT_PIN, LOW);
}

void setup() {
    setupLights(lights);
    pinMode(3, OUTPUT);
}

bool state = false;
void loop() {
    writeLights(lights);
    delay(1000);
    state = !state;
    digitalWrite(3, state ? HIGH : LOW);
}
