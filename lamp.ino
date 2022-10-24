#define LED_PIN 2
#define LED_PATTERN_SIZE 2
#define LED_COUNT 4

#define BTN_PIN 1
#define BTN_IDLE 0
#define BTN_JUST_PRESSED 1
#define BTN_PRESSED 2
#define BTN_LONG_PRESSED 3
#define BTN_RELEASED 4
#define BTN_LONG_RELEASED 5
#define BTN_LONG_TICKS 50

#define STATES 4
#define COLORS 2

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


static const uint8_t COLOR[3 * STATES * LED_PATTERN_SIZE] {
    // stored as RGB

    // blue
    0x00, 0x00, 0x00, // state 0
    0x00, 0x00, 0x00,

    0x00, 0x7F, 0x7F, // state 1
    0x00, 0x00, 0x00,

    0x00, 0x7F, 0x7F, // state 2
    0x00, 0x7F, 0x7F,

    0x00, 0xFF, 0xFF, // state 3
    0x00, 0xFF, 0xFF,
};


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

struct State {
    uint8_t colorState = 0;
    int8_t lightState = -1;
    uint8_t buttonState = BTN_PRESSED; // for init

    uint8_t buttonTicks = 0;
  
} state;

void writeLights(struct Lights &lights) {
    const uint8_t mask = digitalPinToBitMask(LED_PIN);
    volatile uint8_t hi = PORTB | mask;
    volatile uint8_t lo = PORTB & (~mask);
    volatile uint8_t *start = ((uint8_t*) lights.colors);
    volatile uint8_t data = *start;
    volatile uint8_t next = (data & 0x80) ? hi : lo;
    volatile uint8_t colorIndex;
    volatile uint8_t colorCount = 0x00;
    volatile uint8_t ledCount = LED_COUNT / LED_PATTERN_SIZE;
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
        "nop \n\t"                  // 1    nop
        "out %[port], %[lo] \n\t"   // 1    port = LO
        "rjmp .+0 \n\t"             // 2    nop nop
        "nop \n\t"                  // 1    nop

        "out %[port], %[hi] \n\t"   // 1    port = HI
        "rjmp .+0 \n\t"             //      nop nop
        "out %[port], %[next] \n\t" // 1    port = next
        "mov %[next], %[lo] \n\t"   // 1    next = LO
        "sbrc %[data], 5 \n\t"      // 1/2  if data & 0x20
        "mov %[next], %[hi] \n\t"   // 1    next = HI
        "nop \n\t"                  // 1    nop
        "out %[port], %[lo] \n\t"   // 1    port = LO
        "rjmp .+0 \n\t"             // 2    nop nop
        "nop \n\t"                  // 1    nop

        "out %[port], %[hi] \n\t"   // 1    port = HI
        "rjmp .+0 \n\t"             //      nop nop
        "out %[port], %[next] \n\t" // 1    port = next
        "mov %[next], %[lo] \n\t"   // 1    next = LO
        "sbrc %[data], 4 \n\t"      // 1/2  if data & 0x10
        "mov %[next], %[hi] \n\t"   // 1    next = HI
        "nop \n\t"                  // 1    nop
        "out %[port], %[lo] \n\t"   // 1    port = LO
        "rjmp .+0 \n\t"             // 2    nop nop
        "nop \n\t"                  // 1    nop

        "out %[port], %[hi] \n\t"   // 1    port = HI
        "rjmp .+0 \n\t"             //      nop nop
        "out %[port], %[next] \n\t" // 1    port = next
        "mov %[next], %[lo] \n\t"   // 1    next = LO
        "sbrc %[data], 3 \n\t"      // 1/2  if data & 0x08
        "mov %[next], %[hi] \n\t"   // 1    next = HI
        "nop \n\t"                  // 1    nop
        "out %[port], %[lo] \n\t"   // 1    port = LO
        "rjmp .+0 \n\t"             // 2    nop nop
        "nop \n\t"                  // 1    nop

        "out %[port], %[hi] \n\t"   // 1    port = HI
        "rjmp .+0 \n\t"             //      nop nop
        "out %[port], %[next] \n\t" // 1    port = next
        "mov %[next], %[lo] \n\t"   // 1    next = LO
        "sbrc %[data], 2 \n\t"      // 1/2  if data & 0x04
        "mov %[next], %[hi] \n\t"   // 1    next = HI
        "nop \n\t"                  // 1    nop
        "out %[port], %[lo] \n\t"   // 1    port = LO
        "rjmp .+0 \n\t"             // 2    nop nop
        "nop \n\t"                  // 1    nop

        "out %[port], %[hi] \n\t"   // 1    port = HI
        "rjmp .+0 \n\t"             // 2    nop nop
        "out %[port], %[next] \n\t" // 1    port = next
        "mov %[next], %[lo] \n\t"   // 1    next = LO
        "sbrc %[data], 1 \n\t"      // 1/2  if data & 0x02
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
    delay(20);
}

void updateButton(struct State &state) {
    uint8_t pressed = digitalRead(BTN_PIN) == HIGH;

    if (state.buttonState == BTN_IDLE) {
        digitalWrite(3, LOW);
        if (pressed) {
            state.buttonState = BTN_JUST_PRESSED;
            state.buttonTicks = 0;
        }
    } else if (state.buttonState == BTN_JUST_PRESSED) {
        state.buttonState = BTN_PRESSED;
    } else if (state.buttonState == BTN_PRESSED) {
        state.buttonTicks++;
        if (state.buttonTicks >= BTN_LONG_TICKS) {
            state.buttonState = BTN_LONG_PRESSED;
        }
        if (!pressed) {
            state.buttonState = BTN_RELEASED;
        }
    } else if (state.buttonState == BTN_LONG_PRESSED) {
        digitalWrite(3, HIGH);
        if (!pressed) {
            state.buttonState = BTN_LONG_RELEASED;
        }
    } else if (state.buttonState == BTN_RELEASED || state.buttonState == BTN_LONG_RELEASED) {
        state.buttonState = BTN_IDLE;
    }
}

void updateLights(struct State &state, struct Lights &lights) {
    if (state.buttonState == BTN_LONG_PRESSED) {
        state.colorState = (state.colorState + 1) % COLORS;
        
    } else if (state.buttonState == BTN_RELEASED) {
        state.lightState = (state.lightState + 1) % STATES;

        const uint8_t *color = 
            COLOR +                                             // base addr
            state.colorState * 3 * STATES * LED_PATTERN_SIZE +  // color offset
            3 * state.lightState * LED_PATTERN_SIZE;            // state offset

        for (int i = 0; i < LED_PATTERN_SIZE; i++) {
            lights.setColor(i, *(color + 3*i), *(color + 3*i + 1), *(color + 3*i + 2));
        }

        writeLights(lights);
    }
}

void setup() {
    pinMode(LED_PIN, OUTPUT);
    pinMode(BTN_PIN, INPUT);
    pinMode(3, OUTPUT);
}

//bool b = false;
void loop() {
    updateButton(state);
    updateLights(state, lights);
    delay(10);
//    b = !b;
//
//    if (b) {
//        lights.setColor(0, 0xFF, 0x00, 0x00);
//        lights.setColor(1, 0x00, 0xFF, 0x00);
//    } else {
//        lights.setColor(1, 0xFF, 0x00, 0x00);
//        lights.setColor(0, 0x00, 0xFF, 0x00);
//    }
}
