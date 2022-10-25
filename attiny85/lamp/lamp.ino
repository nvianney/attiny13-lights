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

#define STATES 5 // main states accessible by button release
#define STATE_OFF 0
#define STATE_B 1
#define STATE_R 2
#define STATE_A 3
#define STATE_G 4 // gradient
// substate
#define STATE_BR 5 // brightness

#define B_STATE_INC 0
#define B_STATE_WAIT_INC 1
#define B_STATE_WAIT_DEC 2
#define B_STATE_DEC 3
#define B_STATE_WAIT_DURATION 30

#define BR_STATE_SPEED 4

#define G_STATE_SPEED 3


// Vcc = 1.1 * 1023 / ADC => ADC = 1.1 * 1023 / Vcc
#define BATT_MIN_VOLTS 4400 // Vcc = 4.4
#define BATT_MAX_VOLTS 4800 // Vcc = 4.8
#define BATT_IND_MAX 10 // batt indicator max
#define BATT_IND_CLAMP_MIN 2

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
#define CLR_A 0xFF, 0x80, 0x00
#define CLR_BATT_LO 0xA, 0x50, 0x00
#define CLR_BATT_HI 0x00, 0x70, 0x00

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

    uint8_t buttonState = BTN_PRESSED;  // BTN_PRESSED for init
    uint8_t buttonTicks = 0;            // for long-press

    uint8_t brightnessPrevState = 0;    // state prior to entering brightness
    uint8_t brightnessState = 0;        // indicates increasing/decreasing/waiting brightness
    uint8_t brightnessTicks = 0;        // to determine wait time for brightness
    uint16_t brightness = 127;

    uint8_t gradientTick = 0;
  
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
    
    //      min     typ     max
    // T0H  0.25    0.4     0.55
    // T1H  0.65    0.8     0.95
    // T0L  0.7     0.85    1.0
    // T1L  0.3     0.45    0.6
    asm volatile(
        "top: \n\t"
        
        "out %[port], %[hi] \n\t"   // 1    port = HI       0
        "rjmp .+0 \n\t"             //      nop nop         0.125 0.250
        "out %[port], %[next] \n\t" // 1    port = next     0.375
        "mov %[next], %[lo] \n\t"   // 1    next = LO       0.500
        "sbrc %[data], 6 \n\t"      // 1/2  if data & 0x40  0.625
        "mov %[next], %[hi] \n\t"   // 1    next = HI       0.750
        "out %[port], %[lo] \n\t"   // 1    port = LO       0.875
        "nop \n\t"                  // 1    nop             1.000
        "rjmp .+0 \n\t"             // 2    nop nop         1.125 1.250

        "out %[port], %[hi] \n\t"   // 1    port = HI       0
        "rjmp .+0 \n\t"             //      nop nop         0.125 0.250
        "out %[port], %[next] \n\t" // 1    port = next     0.375
        "mov %[next], %[lo] \n\t"   // 1    next = LO       0.500
        "sbrc %[data], 5 \n\t"      // 1/2  if data & 0x20  0.625
        "mov %[next], %[hi] \n\t"   // 1    next = HI       0.750
        "out %[port], %[lo] \n\t"   // 1    port = LO       0.875
        "nop \n\t"                  // 1    nop             1.000
        "rjmp .+0 \n\t"             // 2    nop nop         1.125 1.250

        "out %[port], %[hi] \n\t"   // 1    port = HI       0
        "rjmp .+0 \n\t"             //      nop nop         0.125 0.250
        "out %[port], %[next] \n\t" // 1    port = next     0.375
        "mov %[next], %[lo] \n\t"   // 1    next = LO       0.500
        "sbrc %[data], 4 \n\t"      // 1/2  if data & 0x10  0.625
        "mov %[next], %[hi] \n\t"   // 1    next = HI       0.750
        "out %[port], %[lo] \n\t"   // 1    port = LO       0.875
        "nop \n\t"                  // 1    nop             1.000
        "rjmp .+0 \n\t"             // 2    nop nop         1.125 1.250

        "out %[port], %[hi] \n\t"   // 1    port = HI       0
        "rjmp .+0 \n\t"             //      nop nop         0.125 0.250
        "out %[port], %[next] \n\t" // 1    port = next     0.375
        "mov %[next], %[lo] \n\t"   // 1    next = LO       0.500
        "sbrc %[data], 3 \n\t"      // 1/2  if data & 0x08  0.625
        "mov %[next], %[hi] \n\t"   // 1    next = HI       0.750
        "out %[port], %[lo] \n\t"   // 1    port = LO       0.875
        "nop \n\t"                  // 1    nop             1.000
        "rjmp .+0 \n\t"             // 2    nop nop         1.125 1.250

        "out %[port], %[hi] \n\t"   // 1    port = HI       0
        "rjmp .+0 \n\t"             //      nop nop         0.125 0.250
        "out %[port], %[next] \n\t" // 1    port = next     0.375
        "mov %[next], %[lo] \n\t"   // 1    next = LO       0.500
        "sbrc %[data], 2 \n\t"      // 1/2  if data & 0x04  0.625
        "mov %[next], %[hi] \n\t"   // 1    next = HI       0.750
        "out %[port], %[lo] \n\t"   // 1    port = LO       0.875
        "nop \n\t"                  // 1    nop             1.000
        "rjmp .+0 \n\t"             // 2    nop nop         1.125 1.250

        "out %[port], %[hi] \n\t"   // 1    port = HI       0
        "rjmp .+0 \n\t"             // 2    nop nop         0.125 0.250
        "out %[port], %[next] \n\t" // 1    port = next     0.375
        "mov %[next], %[lo] \n\t"   // 1    next = LO       0.500
        "sbrc %[data], 1 \n\t"      // 1/2  if data & 0x02  0.625
        "mov %[next], %[hi] \n\t"   // 1      next = HI     0.750
        "out %[port], %[lo] \n\t"   // 1    port = LO       0.875
        "lsr %[ci] \n\t"            // 1    light >> 1      1.000
        "sbrs %[ci], 0 \n\t"        // 1/2  if light == 0   1.125
        "subi %[lc], 1 \n\t"        // 1      count--       1.250
        

        "out %[port], %[hi] \n\t"   // 1    port = HI               0
        "sbrs %[ci], 0 \n\t"        // 1/2  if light == 0           0.125  (not literally, but equivalent for our scenario)
        "mov %[ptr], %[start] \n\t" // 1      ptr = start           0.250
        "out %[port], %[next] \n\t" // 1    port = next             0.375
        "sbrs %[ci], 0 \n\t"        // 1/2  if light == 0           0.500
        "mov %[ci], %[cc] \n\t"     // 1      light = colorCount    0.625        
        "sbrs %[ci], 0 \n\t"        // 1/2  if light == 0           0.750
        "out %[port], %[lo] \n\t"   // 1    port = LO               0.875
        "mov %[next], %[lo] \n\t"   // 1    next = LO               1.000
        "sbrc %[data], 0 \n\t"      // 1/2  if data & 0x01          1.125
        "mov %[next], %[hi] \n\t"   // 1      next = HI             1.250

        "out %[port], %[hi] \n\t"   // 1    port = HI       0
        "ld %[data], %a[ptr]+ \n\t" // 2    data = *ptr++   0.125
        "nop \n\t"                  // 1    nop             0.250
        "out %[port], %[next] \n\t" // 1    port = next     0.375
        "mov %[next], %[lo] \n\t"   // 1    next = LO       0.500
        "sbrc %[data], 7 \n\t"      // 1/2  if data & 0x80  0.625
        "mov %[next], %[hi] \n\t"   // 1      next = HI     0.750
        "out %[port], %[lo] \n\t"   // 1    port = LO       0.875
        "cpse %[lc], %[zero] \n\t"  // 1/2  if count != 0   1.000
        "rjmp top \n\t"             // 2    goto top        1.125 1.250

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
    ADMUX = _BV(MUX3) | _BV(MUX2);

    _delay_ms(2); // Wait for Vref to settle
    ADCSRA |= _BV(ADSC); // Start conversion
    while (bit_is_set(ADCSRA, ADSC)); // measuring

    uint32_t low  = ADCL; // must read ADCL first - it then locks ADCH  
    uint32_t high = ADCH; // unlocks both

    uint32_t result = (high<<8) | low;

    result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
    return result; // Vcc in millivolts
}

inline void set(uint8_t *colors, uint8_t r, uint8_t g, uint8_t b) {
    *(colors) = r;
    *(colors+1) = g;
    *(colors+2) = b;
}

/*
 * [0, 1, 2, 3] = [off, blue, red, amber]
 * 
 * Q = {0, 1, 2, 3}
 * q_0 = 0
 * d(0,)
 *
 */
bool updateLights(struct State &state, struct Lights &lights) {
    bool dirty = false;

    if (state.lightState == STATE_OFF) {
        if (state.buttonState == BTN_JUST_LONG_PRESSED) {
            const uint16_t delta = BATT_MAX_VOLTS - BATT_MIN_VOLTS;

            // map from [x, y] to [0, b] without using floats. 100 gives three significant figures.
            uint32_t progress = ((vcr() - BATT_MIN_VOLTS) * 100 * BATT_IND_MAX) / (delta * 100);

            // clamp lower value. don't want indicator to be 0 at a very low battery.
            if (progress < BATT_IND_CLAMP_MIN) progress = BATT_IND_CLAMP_MIN;
            for (uint8_t i = 0; i < LED_PATTERN_SIZE; i++) {
                lights.setColor(i, 0x00, 0xA0, 0x00);
            }
            lights.count = progress;
            writeLights(lights);

        } else if (state.buttonState == BTN_LONG_RELEASED) {
            dirty = true;
            lights.count = LED_COUNT;

        } else if (state.buttonState == BTN_RELEASED) {
            dirty = true;
            state.lightState++;
        }

    } else if (state.lightState == STATE_BR) {
        dirty = true;

        if (state.buttonState == BTN_RELEASED) {
            state.lightState = state.brightnessPrevState;

        } else {
            if (state.brightnessState == B_STATE_INC) {
                state.brightness += BR_STATE_SPEED;

            } else if (state.brightnessState == B_STATE_DEC) {
                state.brightness -= BR_STATE_SPEED;

            } else { // waiting
                state.brightnessTicks += 1;

                if (state.brightnessTicks >= B_STATE_WAIT_DURATION) {
                    if (state.brightnessState == B_STATE_WAIT_INC) {
                        state.brightnessState = B_STATE_INC;
                    } else if (state.brightnessState == B_STATE_WAIT_DEC) {
                        state.brightnessState = B_STATE_DEC;
                    }
                }
            }

            if (state.brightness < 0x01) {
                state.brightness = 0x01;
                state.brightnessState = B_STATE_WAIT_INC;
                state.brightnessTicks = 0;
            } else if (state.brightness > 0xFF) {
                state.brightness = 0xFF;
                state.brightnessState = B_STATE_WAIT_DEC;
                state.brightnessTicks = 0;
            }
        }

    } else { // STATE_[B|R|A|G]
        if (state.lightState == STATE_G) {
            // animate color

        }

        if (state.buttonState == BTN_JUST_LONG_PRESSED) {
            state.brightnessPrevState = state.lightState;
            state.lightState = STATE_BR;

        } else if (state.buttonState == BTN_RELEASED) {
            state.lightState++;
        }
    }

    if (state.lightState >= STATES) state.lightState = 0;
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
        sleep();
    }
}
