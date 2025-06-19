// Smart Traffic Light Controller for AVR (ATmega328P)
// Advanced: Debouncing, latching, state machine, timer interrupts, blinking, UART debug
// SimulIDE/KiCad compatible, no external dependencies

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>

// Pin definitions (PORTD: inputs, PORTB: outputs)
#define NS_VEHICLE_SENSOR_PIN PD2
#define EW_VEHICLE_SENSOR_PIN PD3
#define NS_PED_BUTTON_PIN     PD4
#define EW_PED_BUTTON_PIN     PD5
#define EMERGENCY_OVERRIDE_PIN PD6

#define NS_VEHICLE_SENSOR   debounce_button(NS_VEHICLE_SENSOR_PIN)
#define EW_VEHICLE_SENSOR   debounce_button(EW_VEHICLE_SENSOR_PIN)
#define NS_PED_BUTTON       debounce_button(NS_PED_BUTTON_PIN)
#define EW_PED_BUTTON       debounce_button(EW_PED_BUTTON_PIN)
#define EMERGENCY_OVERRIDE  debounce_button(EMERGENCY_OVERRIDE_PIN)

// Output macros for lights
#define NS_RED_ON     (PORTB |= (1 << PB0))
#define NS_RED_OFF    (PORTB &= ~(1 << PB0))
#define NS_YELLOW_ON  (PORTB |= (1 << PB1))
#define NS_YELLOW_OFF (PORTB &= ~(1 << PB1))
#define NS_GREEN_ON   (PORTB |= (1 << PB2))
#define NS_GREEN_OFF  (PORTB &= ~(1 << PB2))
#define EW_RED_ON     (PORTB |= (1 << PB3))
#define EW_RED_OFF    (PORTB &= ~(1 << PB3))
#define EW_YELLOW_ON  (PORTB |= (1 << PB4))
#define EW_YELLOW_OFF (PORTB &= ~(1 << PB4))
#define EW_GREEN_ON   (PORTB |= (1 << PB5))
#define EW_GREEN_OFF  (PORTB &= ~(1 << PB5))
#define NS_PED_ON     (PORTB |= (1 << PB6))
#define NS_PED_OFF    (PORTB &= ~(1 << PB6))
#define EW_PED_ON     (PORTB |= (1 << PB7))
#define EW_PED_OFF    (PORTB &= ~(1 << PB7))

// Timing constants (ms)
#define GREEN_MIN 5000UL
#define YELLOW_TIME 2000UL
#define PED_TIME 4000UL
#define PED_BLINK_TIME 1000UL
#define PED_BLINK_INTERVAL 250UL
#define DEBOUNCE_TIME 30UL

// UART baud rate
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1

// State machine states
typedef enum {
    STATE_NS_GREEN,
    STATE_NS_YELLOW,
    STATE_NS_PED,
    STATE_EW_GREEN,
    STATE_EW_YELLOW,
    STATE_EW_PED,
    STATE_ALL_RED,
    STATE_EMERGENCY
} TrafficState;

// Latch variables for pedestrian requests
volatile bool ns_ped_request = false;
volatile bool ew_ped_request = false;

// Millisecond tick
volatile uint32_t ms_ticks = 0;

// UART functions
void uart_init(unsigned int ubrr) {
    UBRR0H = (unsigned char)(ubrr>>8);
    UBRR0L = (unsigned char)ubrr;
    UCSR0B = (1<<TXEN0);
    UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);
}
void uart_putchar(char c) {
    while (!(UCSR0A & (1<<UDRE0)));
    UDR0 = c;
}
void uart_puts(const char* s) {
    while (*s) uart_putchar(*s++);
}
void uart_putnum(uint32_t n) {
    char buf[11];
    int i=10;
    buf[i--]=0;
    if(n==0) buf[i--]='0';
    while(n && i>=0) { buf[i--]='0'+(n%10); n/=10; }
    uart_puts(&buf[i+1]);
}

// Debounce function for buttons/sensors
bool debounce_button(uint8_t pin) {
    static uint8_t last_state[8] = {0};
    static uint32_t last_time[8] = {0};
    bool pressed = !(PIND & (1 << pin)); // Active low
    if (pressed != last_state[pin]) {
        last_time[pin] = ms_ticks;
        last_state[pin] = pressed;
    }
    if ((ms_ticks - last_time[pin]) > DEBOUNCE_TIME) {
        return pressed;
    }
    return false;
}

void init_io() {
    // Inputs: PD2-PD6
    DDRD &= ~((1 << NS_VEHICLE_SENSOR_PIN)|(1 << EW_VEHICLE_SENSOR_PIN)|
              (1 << NS_PED_BUTTON_PIN)|(1 << EW_PED_BUTTON_PIN)|(1 << EMERGENCY_OVERRIDE_PIN));
    PORTD |= (1 << NS_VEHICLE_SENSOR_PIN)|(1 << EW_VEHICLE_SENSOR_PIN)|
              (1 << NS_PED_BUTTON_PIN)|(1 << EW_PED_BUTTON_PIN)|(1 << EMERGENCY_OVERRIDE_PIN); // Pull-ups
    // Outputs: PB0-PB7
    DDRB |= 0xFF;
    PORTB = 0x00;
}

void set_ns_green() {
    NS_GREEN_ON; NS_YELLOW_OFF; NS_RED_OFF;
    EW_RED_ON; EW_YELLOW_OFF; EW_GREEN_OFF;
    NS_PED_OFF; EW_PED_OFF;
}

void set_ns_yellow() {
    NS_GREEN_OFF; NS_YELLOW_ON; NS_RED_OFF;
    EW_RED_ON; EW_YELLOW_OFF; EW_GREEN_OFF;
    NS_PED_OFF; EW_PED_OFF;
}

void set_ew_green() {
    EW_GREEN_ON; EW_YELLOW_OFF; EW_RED_OFF;
    NS_RED_ON; NS_YELLOW_OFF; NS_GREEN_OFF;
    NS_PED_OFF; EW_PED_OFF;
}

void set_ew_yellow() {
    EW_GREEN_OFF; EW_YELLOW_ON; EW_RED_OFF;
    NS_RED_ON; NS_YELLOW_OFF; NS_GREEN_OFF;
    NS_PED_OFF; EW_PED_OFF;
}

void set_all_red() {
    NS_RED_ON; NS_YELLOW_OFF; NS_GREEN_OFF;
    EW_RED_ON; EW_YELLOW_OFF; EW_GREEN_OFF;
    NS_PED_OFF; EW_PED_OFF;
}

void blink_ped_light(bool ns, bool on) {
    if (ns) {
        if (on) NS_PED_ON; else NS_PED_OFF;
    } else {
        if (on) EW_PED_ON; else EW_PED_OFF;
    }
}

void allow_ns_pedestrian() {
    set_all_red();
    NS_PED_ON;
    _delay_ms(PED_TIME);
    NS_PED_OFF;
    ns_ped_request = false;
}

void allow_ew_pedestrian() {
    set_all_red();
    EW_PED_ON;
    _delay_ms(PED_TIME);
    EW_PED_OFF;
    ew_ped_request = false;
}

void emergency_mode() {
    set_all_red();
    NS_GREEN_ON;
    uart_puts("EMERGENCY MODE\r\n");
    while (debounce_button(EMERGENCY_OVERRIDE_PIN)) {
        // Stay in emergency mode as long as override is active
    }
    set_all_red();
    uart_puts("EMERGENCY CLEARED\r\n");
}

ISR(TIMER1_COMPA_vect) {
    ms_ticks++;
}

int main(void) {
    init_io();
    uart_init(MYUBRR);
    // Timer1 CTC mode, 1ms interrupt
    TCCR1B |= (1 << WGM12);
    OCR1A = (F_CPU / 64 / 1000) - 1;
    TIMSK1 |= (1 << OCIE1A);
    TCCR1B |= (1 << CS11) | (1 << CS10); // prescaler 64
    sei();

    TrafficState state = STATE_NS_GREEN;
    uint32_t state_start = ms_ticks;
    bool ped_blink_on = true;
    uint32_t ped_blink_last = 0;

    while (1) {
        // Latch pedestrian requests
        if (NS_PED_BUTTON) ns_ped_request = true;
        if (EW_PED_BUTTON) ew_ped_request = true;

        // Emergency override
        if (EMERGENCY_OVERRIDE) {
            emergency_mode();
            state = STATE_NS_GREEN;
            state_start = ms_ticks;
            continue;
        }

        switch (state) {
            case STATE_NS_GREEN:
                set_ns_green();
                if ((ms_ticks - state_start) >= GREEN_MIN) {
                    if (ns_ped_request) {
                        state = STATE_NS_PED;
                        state_start = ms_ticks;
                        uart_puts("NS PED\r\n");
                    } else {
                        state = STATE_NS_YELLOW;
                        state_start = ms_ticks;
                        uart_puts("NS YELLOW\r\n");
                    }
                }
                break;
            case STATE_NS_PED:
                set_all_red();
                // Blinking last second
                if ((ms_ticks - state_start) < (PED_TIME - PED_BLINK_TIME)) {
                    NS_PED_ON;
                } else {
                    if ((ms_ticks - ped_blink_last) >= PED_BLINK_INTERVAL) {
                        ped_blink_on = !ped_blink_on;
                        ped_blink_last = ms_ticks;
                    }
                    blink_ped_light(true, ped_blink_on);
                }
                if ((ms_ticks - state_start) >= PED_TIME) {
                    NS_PED_OFF;
                    ns_ped_request = false;
                    state = STATE_NS_YELLOW;
                    state_start = ms_ticks;
                    uart_puts("NS PED DONE\r\n");
                }
                break;
            case STATE_NS_YELLOW:
                set_ns_yellow();
                if ((ms_ticks - state_start) >= YELLOW_TIME) {
                    state = STATE_ALL_RED;
                    state_start = ms_ticks;
                    uart_puts("ALL RED\r\n");
                }
                break;
            case STATE_ALL_RED:
                set_all_red();
                if ((ms_ticks - state_start) >= 500) {
                    if (EW_VEHICLE_SENSOR) {
                        state = STATE_EW_GREEN;
                        uart_puts("EW GREEN\r\n");
                    } else {
                        state = STATE_NS_GREEN;
                        uart_puts("NS GREEN\r\n");
                    }
                    state_start = ms_ticks;
                }
                break;
            case STATE_EW_GREEN:
                set_ew_green();
                if ((ms_ticks - state_start) >= GREEN_MIN) {
                    if (ew_ped_request) {
                        state = STATE_EW_PED;
                        state_start = ms_ticks;
                        uart_puts("EW PED\r\n");
                    } else {
                        state = STATE_EW_YELLOW;
                        state_start = ms_ticks;
                        uart_puts("EW YELLOW\r\n");
                    }
                }
                break;
            case STATE_EW_PED:
                set_all_red();
                // Blinking last second
                if ((ms_ticks - state_start) < (PED_TIME - PED_BLINK_TIME)) {
                    EW_PED_ON;
                } else {
                    if ((ms_ticks - ped_blink_last) >= PED_BLINK_INTERVAL) {
                        ped_blink_on = !ped_blink_on;
                        ped_blink_last = ms_ticks;
                    }
                    blink_ped_light(false, ped_blink_on);
                }
                if ((ms_ticks - state_start) >= PED_TIME) {
                    EW_PED_OFF;
                    ew_ped_request = false;
                    state = STATE_EW_YELLOW;
                    state_start = ms_ticks;
                    uart_puts("EW PED DONE\r\n");
                }
                break;
            case STATE_EW_YELLOW:
                set_ew_yellow();
                if ((ms_ticks - state_start) >= YELLOW_TIME) {
                    state = STATE_ALL_RED;
                    state_start = ms_ticks;
                    uart_puts("ALL RED\r\n");
                }
                break;
            default:
                state = STATE_NS_GREEN;
                state_start = ms_ticks;
                break;
        }
    }
    return 0;
} 