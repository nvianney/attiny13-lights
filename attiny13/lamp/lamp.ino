#define LED_MOSFET_PIN PB0
#define BTN_PIN PB1
#define LED_PIN PB2

#define BTN_INTERRUPT PCINT1

#define LED_PATTERN_SIZE 1
#define LED_COUNT 4

#define BTN_IDLE 0
#define BTN_JUST_PRESSED 1
#define BTN_PRESSED 2
#define BTN_JUST_LONG_PRESSED 3
#define BTN_LONG_PRESSED 4
#define BTN_RELEASED 5
#define BTN_LONG_RELEASED 6
#define BTN_LONG_TICKS 50

#define STATES 4
#define COLORS 3

#define BRIGHTNESS_SPEED 4

// Vcc = 1.1 * 1023 / ADC => ADC = 1.1 * 1023 / Vcc
//#define MAX_VOLTAGE_ADC 256 // Vcc = 4.4
//#define MIN_VOLTAGE_ADC 234 // Vcc = 4.8
#define MAX_VOLTAGE_ADC 350 // Vcc = 3.2
#define MIN_VOLTAGE_ADC 330 // Vcc = 3.1
// #define VOLTAGE_INC 10

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

#define CLR_B 0x00, 0xFF, 0xFF
#define CLR_R 0xFF, 0x20, 0x20
#define CLR_A 0xFF, 0xFF, 0x00

// using this macro still results in the same no. of bytes used.
//#define SET_COLOR(ptr, r, g, b) \
//    *(ptr) = r; \
//    *(ptr + 1) = g; \
//    *(ptr + 2) = b;

#include <avr/sleep.h>

struct Lights {
    uint16_t count = LED_COUNT;
    uint8_t colors[3 * LED_PATTERN_SIZE];

    void setColor(uint8_t index, uint8_t red, uint8_t green, uint8_t blue) {
        const uint8_t offset = 3 * index;
        colors[offset + 0] = green;
        colors[offset + 1] = red;
        colors[offset + 2] = blue;
    }
} lights;

struct State {
    int8_t lightState = -1;
    uint8_t buttonState = BTN_PRESSED; // for init
    uint8_t buttonTicks = 0;

    int8_t brightnessDirection = BRIGHTNESS_SPEED;
    uint16_t brightness = 127;
  
} state;

void writeLights(struct Lights &lights) {
    const uint8_t mask = 1 << PB2;
    volatile uint8_t hi = PORTB | mask;
    volatile uint8_t lo = PORTB & (~mask);
    volatile uint8_t *start = ((uint8_t*) lights.colors);
    volatile uint8_t data = *start;
    volatile uint8_t next = (data & 0x80) ? hi : lo;
    volatile uint8_t colorIndex = 0b111; // adjust this depending on # of unique colors. pattern=1: 0b111. pattern=2: 0b111111
    volatile uint8_t colorCount = colorIndex;
    volatile uint8_t ledCount = lights.count / LED_PATTERN_SIZE;
    volatile uint8_t zero = 0;

    // for (int i = 0; i < LED_PATTERN_SIZE * 3; i++) { // TODO: eliminate for-loop
    //     colorCount <<= 1;
    //     colorCount |= 0x01;
    // }
    // colorIndex = colorCount;
    
    asm volatile(
        "top: \n\t"
        
        "out %[port], %[hi] \n\t"   // 1    port = HI       0
        "rjmp .+0 \n\t"             //      nop nop         0.125 0.250
        "out %[port], %[next] \n\t" // 1    port = next     0.375
        "mov %[next], %[lo] \n\t"   // 1    next = LO       0.500
        "sbrc %[data], 6 \n\t"      // 1/2  if data & 0x40  0.625
        "mov %[next], %[hi] \n\t"   // 1    next = HI       0.750
        "nop \n\t"                  // 1    nop             0.875
        "out %[port], %[lo] \n\t"   // 1    port = LO       1.000
        "rjmp .+0 \n\t"             // 2    nop nop         1.125
        "nop \n\t"                  // 1    nop             1.125

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
    _delay_ms(20);
}

void updateButton(struct State &state) {
    uint8_t pressed = PINB & (1 << PB1);

    if (state.buttonState == BTN_IDLE) {
        if (pressed) {
            state.buttonState = BTN_JUST_PRESSED;
        }
    } else if (state.buttonState == BTN_JUST_PRESSED) {
        state.buttonState = BTN_PRESSED;
        state.buttonTicks = 0;
    } else if (state.buttonState == BTN_PRESSED) {
        state.buttonTicks++;
        if (state.buttonTicks >= BTN_LONG_TICKS) {
            state.buttonState = BTN_JUST_LONG_PRESSED;
        }
        if (!pressed) {
            state.buttonState = BTN_RELEASED;
        }
    } else if (state.buttonState == BTN_JUST_LONG_PRESSED) {
        state.buttonState = BTN_LONG_PRESSED;
    } else if (state.buttonState == BTN_LONG_PRESSED) {
        if (!pressed) {
            state.buttonState = BTN_LONG_RELEASED;
        }
    } else /*if (state.buttonState == BTN_RELEASED || state.buttonState == BTN_LONG_RELEASED)*/ {
        state.buttonState = BTN_IDLE;
    }
}

uint32_t vcr() {
    // https://provideyourown.com/2012/secret-arduino-voltmeter-measure-battery-voltage/
    ADMUX = _BV(REFS0);

    _delay_ms(2); // Wait for Vref to settle
    ADCSRA |= _BV(ADSC); // Start conversion
    while (bit_is_set(ADCSRA, ADSC)); // measuring

    uint32_t low  = ADCL; // must read ADCL first - it then locks ADCH  
    uint32_t high = ADCH; // unlocks both

    uint32_t result = (high<<8) | low;

    // result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
    // return result; // Vcc in millivolts  

    return result; // ADC = 1.1 * 1023 / Vcc
}

inline void set(uint8_t *colors, uint8_t r, uint8_t g, uint8_t b) {
    *(colors) = r;
    *(colors+1) = g;
    *(colors+2) = b;
}

bool updateLights(struct State &state, struct Lights &lights) {
    bool dirty = false;
    if (state.buttonState == BTN_JUST_LONG_PRESSED) {
        // previously, we handle battery level here. to save .text space, we moved it to BTN_LONG_PRESS
    } else if (state.buttonState == BTN_LONG_PRESSED) {
        if (state.lightState == 0) {
            // const uint32_t delta = MAX_VOLTAGE_ADC - MIN_VOLTAGE_ADC;
            // dv = 0.0 -> ADC = 0
            // dv = 0.4 -> ADC = 22
            uint32_t value = MAX_VOLTAGE_ADC - (vcr() - MIN_VOLTAGE_ADC); // Vcc - Vref ADC
            
            // delta is 256-234 = 22. may need to double value.
            for (uint8_t i = 0; i < LED_PATTERN_SIZE; i++) {
                lights.setColor(i, 0x00, 0xFF, 0x00);
            }
            lights.count = value >> 7;
            writeLights(lights);
        } else {
            dirty = true;
            int16_t output = state.brightness + state.brightnessDirection;
            if (output < 0x01) {
                state.brightness = 0x01;
            } else if (output > 0xFF) {
                state.brightness = 0xFF;
            } else {
                state.brightness = output;
            }
        }

    } else if (state.buttonState == BTN_LONG_RELEASED) {
        if (state.lightState == 0) {
            dirty = true;
            lights.count = LED_COUNT;
        } else {
            state.brightnessDirection = -state.brightnessDirection;
        }

    } else if (state.buttonState == BTN_RELEASED) {
        state.lightState++;
        if (state.lightState >= STATES) state.lightState = 0;
        dirty = true;
    }

    if (!dirty) return true;

    uint8_t colors[3];
    if (state.lightState == 0) {
        set(colors, 0x00, 0x00, 0x00);
    } else if (state.lightState == 1) {
        set(colors, CLR_B);
    } else if (state.lightState == 2) {
        set(colors, CLR_R);
    } else if (state.lightState == 3) {
        set(colors, CLR_A);
    }

    uint8_t red = ((uint16_t) colors[0]) * state.brightness / 0xFF;
    uint8_t green = ((uint16_t) colors[1]) * state.brightness / 0xFF;
    uint8_t blue = ((uint16_t) colors[2]) * state.brightness / 0xFF;
    for (uint8_t i = 0; i < LED_PATTERN_SIZE; i++) {
        lights.setColor(i, red, green, blue);
    }
    writeLights(lights);

    return true;
}

void setup() {
    // 1 is output
    DDRB |= (1 << LED_PIN);
}

ISR(PCINT0_vect){}

void sleep() {
    // https://bigdanzblog.wordpress.com/2014/08/10/attiny85-wake-from-sleep-on-pin-state-change-code-example/comment-page-1/
    GIMSK |= _BV(PCIE);                     // Enable Pin Change Interrupts
    PCMSK |= _BV(BTN_INTERRUPT);            // Use PB3 as interrupt pin
    ADCSRA &= ~_BV(ADEN);                   // ADC off
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);    // replaces above statement

    sleep_enable();                         // Sets the Sleep Enable bit in the MCUCR Register (SE BIT)
    sei();                                  // Enable interrupts
    sleep_cpu();                            // sleep

    cli();                                  // Disable interrupts
    PCMSK &= ~_BV(BTN_INTERRUPT);           // Turn off PB3 as interrupt pin
    sleep_disable();                        // Clear SE bit
    ADCSRA |= _BV(ADEN);                    // ADC on

    sei();                                  // Enable interrupts
}

void loop() {
    updateButton(state);
    bool canSleep = updateLights(state, lights);
    _delay_ms(10);

    if (state.buttonState == BTN_IDLE && canSleep) {
//        sleep();
    }
}
