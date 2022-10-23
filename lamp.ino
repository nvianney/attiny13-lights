#define LED_PIN 2
#define LED_PATTERN_SIZE 2
#define LED_COUNT 4
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
    uint8_t count = 1;
    
    uint8_t colors[3 * LED_PATTERN_SIZE];

    void setColor(uint8_t index, uint8_t red, uint8_t green, uint8_t blue) {
        const uint8_t offset = 3 * index;
        colors[offset + 0] = green;
        colors[offset + 1] = red;
        colors[offset + 2] = blue;
    }
} lights;

void setupLights(struct Lights &lights) {
    lights.setColor(0, 0x00, 0xFF, 0x00);
    lights.setColor(1, 0xFF, 0x00, 0x00);

    pinMode(LED_PIN, OUTPUT);
}

void writeLights(struct Lights &lights) {
    const uint8_t mask = digitalPinToBitMask(LED_PIN);
    volatile uint8_t hi = PORTB | mask;
    volatile uint8_t lo = PORTB & (~mask);
    volatile uint8_t *start = ((uint8_t*) lights.colors);
    volatile uint8_t data = *start;
    volatile uint8_t next = (data & 0x80) ? hi : lo;
    volatile uint8_t colorIndex;
    volatile uint8_t colorCount = 0x00;
    volatile uint8_t ledCount = LED_COUNT;
    volatile uint8_t zero = 0;

    for (int i = 0; i < LED_PATTERN_SIZE * 3; i++) {
        colorCount <<= 1;
        colorCount |= 0x01;
    }
    colorIndex = colorCount;
    

    asm volatile(
        "top: \n\t"
        
        "out %[port], %[hi] \n\t"   // 1    port = HI
        "rjmp .+0 \n\t"             //      nop nop
        "out %[port], %[next] \n\t" // 1    port = next
        "mov %[next], %[lo] \n\t"   // 1    next = LO
        "sbrc %[data], 6 \n\t"      // 1/2  if data & 0x40
        "mov %[next], %[hi] \n\t"   // 1    next = HI
        "nop \n\t"
        "out %[port], %[lo] \n\t"
        "rjmp .+0 \n\t"
        "nop \n\t"

        "out %[port], %[hi] \n\t"   // 1    port = HI
        "rjmp .+0 \n\t"             //      nop nop
        "out %[port], %[next] \n\t" // 1    port = next
        "mov %[next], %[lo] \n\t"   // 1    next = LO
        "sbrc %[data], 5 \n\t"      // 1/2  if data & 0x40
        "mov %[next], %[hi] \n\t"   // 1    next = HI
        "nop \n\t"
        "out %[port], %[lo] \n\t"
        "rjmp .+0 \n\t"
        "nop \n\t"

        "out %[port], %[hi] \n\t"   // 1    port = HI
        "rjmp .+0 \n\t"             //      nop nop
        "out %[port], %[next] \n\t" // 1    port = next
        "mov %[next], %[lo] \n\t"   // 1    next = LO
        "sbrc %[data], 4 \n\t"      // 1/2  if data & 0x40
        "mov %[next], %[hi] \n\t"   // 1    next = HI
        "nop \n\t"
        "out %[port], %[lo] \n\t"
        "rjmp .+0 \n\t"
        "nop \n\t"

        "out %[port], %[hi] \n\t"   // 1    port = HI
        "rjmp .+0 \n\t"             //      nop nop
        "out %[port], %[next] \n\t" // 1    port = next
        "mov %[next], %[lo] \n\t"   // 1    next = LO
        "sbrc %[data], 3 \n\t"      // 1/2  if data & 0x40
        "mov %[next], %[hi] \n\t"   // 1    next = HI
        "nop \n\t"
        "out %[port], %[lo] \n\t"
        "rjmp .+0 \n\t"
        "nop \n\t"

        "out %[port], %[hi] \n\t"   // 1    port = HI
        "rjmp .+0 \n\t"             //      nop nop
        "out %[port], %[next] \n\t" // 1    port = next
        "mov %[next], %[lo] \n\t"   // 1    next = LO
        "sbrc %[data], 2 \n\t"      // 1/2  if data & 0x40
        "mov %[next], %[hi] \n\t"   // 1    next = HI
        "nop \n\t"
        "out %[port], %[lo] \n\t"
        "rjmp .+0 \n\t"
        "nop \n\t"

        "out %[port], %[hi] \n\t"   // 1    port = HI
        "rjmp .+0 \n\t"             // 2    nop nop
        "out %[port], %[next] \n\t" // 1    port = next
        "mov %[next], %[lo] \n\t"   // 1    next = LO
        "sbrc %[data], 1 \n\t"      // 1/2  if data & 0x40
        "mov %[next], %[hi] \n\t"   // 1      next = HI
        "nop \n\t"                  // 1    nop
        "out %[port], %[lo] \n\t"   // 1    port = LO
        "rjmp .+0 \n\t"             // 2    nop nop
        "lsr %[ci] \n\t"            // 1    light >> 1

        "out %[port], %[hi] \n\t"   // 1    port = HI
        "sbrs %[ci], 0 \n\t"        // 1/2  if light == 0
        "subi %[lc], 1 \n\t"        // 1      count--
        "out %[port], %[next] \n\t" // 1    port = next
        "sbrs %[ci], 0 \n\t"        // 1/2  if light == 0 (not literally, but equivalent for our scenario)
        "mov %[ptr], %[start] \n\t" // 1      ptr = start
        "sbrs %[ci], 0 \n\t"        // 1/2  if light == 0
        "mov %[ci], %[cc] \n\t"     // 1      light = colorCount
        "out %[port], %[lo] \n\t"   // 1    port = LO
        "mov %[next], %[lo] \n\t"   // 1    next = LO
        "sbrc %[data], 0 \n\t"      // 1/2  if data & 0x01
        "mov %[next], %[hi] \n\t"   // 1      next = HI

        "out %[port], %[hi] \n\t"   // 1    port = HI
        "ld %[data], %a[ptr]+ \n\t" // 2    data = *ptr++
        "out %[port], %[next] \n\t" // 1    port = next
        "nop \n\t"                  // 1    nop
        "mov %[next], %[lo] \n\t"   // 1    next = LO
        "sbrc %[data], 7 \n\t"      // 1/2  if data & 0x80
        "mov %[next], %[hi] \n\t"   // 1      next = HI
        "out %[port], %[lo] \n\t"   // 1    port = LO
        "cpse %[lc], %[zero] \n\t"  // 1/2  if count != 0
        "rjmp top \n\t"             // 2    goto top

        : [next] "+r" (next),
          [ci] "+r" (colorIndex),
          [lc] "+r" (ledCount)
        : [port] "I" (_SFR_IO_ADDR(PORTB)),
          [hi] "r" (hi),
          [lo] "r" (lo),
          [data] "r" (data),
          [cc] "r" (colorCount),
          [zero] "r" (zero),
          [start] "e" (start),
          [ptr] "e" (start+1)
    );
    digitalWrite(LED_PIN, LOW);
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
