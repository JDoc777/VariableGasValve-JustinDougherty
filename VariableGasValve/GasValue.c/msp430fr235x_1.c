#include <msp430.h>
#include <math.h>

// Pin Definitions
#define CALL_FOR_HEAT     BIT2     // P1.2 (input)
#define IGNITOR_LED       BIT0     // P2.0 (output)
#define PILOT_VALVE       BIT5     // P2.5 (output)
#define SERVO_PWM         BIT0     // P5.0 (TimerB1.1 output)

#define STATUS_RED        BIT0     // P6.0 RED
#define STATUS_GREEN      BIT1     // P6.1 GREEN
#define STATUS_BLUE       BIT2     // P6.2 BLUE

// ADC Input Pin Definitions
#define THERMOCOUPLE_CH   3        // A3 = P1.3
#define THERMISTOR_CH     4        // A4 = P1.4
#define SETPOINT_CH       5        // A5 = P1.5

// Status Enum
enum {STANDBY, HEATING, UP_TO_TEMP};

// Global Variables
volatile unsigned int boiler_temp = 0;              // Boiler temp variable (int)
volatile unsigned int set_point = 0;                // Set point variable (int)
volatile unsigned int flame_detected = 0;           // Flame detect variable (int)
volatile unsigned char call_for_heat_flag = 0;      // Call for heat flag (char)

// Declare Functions
void init_GPIO();
void init_ADC();
void init_TimerB_PWM();
void update_status_LED(unsigned char status);
void turn_everything_off();
void ignition_sequence();
unsigned int read_ADC(unsigned int channel);
float thermocouple_to_temperature(unsigned int adc_value);
float setpoint_to_temperature(unsigned int adc_value);
float thermistor_to_temperature(unsigned int adc_value);

// Main Loop
void main(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // Stop watchdog timer

    init_GPIO();                // Initialize GPIO
    init_ADC();                 // Initialize ADC
    init_TimerB_PWM();          // Initialize PWM timer

    PM5CTL0 &= ~LOCKLPM5;       // Unlock GPIOs

    __enable_interrupt();

    while (1)
    {
        boiler_temp = (unsigned int)thermistor_to_temperature(read_ADC(THERMISTOR_CH));
        set_point = (unsigned int)setpoint_to_temperature(read_ADC(SETPOINT_CH));

        // boiler_temp = (unsigned int)thermistor_to_temperature(read_ADC(THERMISTOR_CH));

        if (boiler_temp >= set_point) {
            turn_everything_off();
            update_status_LED(UP_TO_TEMP);
            continue;
        }

        if (call_for_heat_flag && (boiler_temp < set_point)) {
            update_status_LED(HEATING);
            ignition_sequence();
        } else {
            update_status_LED(STANDBY);
        }

        __delay_cycles(100000); // Polling delay
    }
}

void init_GPIO()
{
    // Input Call for Heat
    P1DIR &= ~CALL_FOR_HEAT;
    P1REN |= CALL_FOR_HEAT;
    P1OUT |= CALL_FOR_HEAT;
    P1IES &= ~CALL_FOR_HEAT;
    P1IFG &= ~CALL_FOR_HEAT;
    P1IE  |= CALL_FOR_HEAT;

    // Outputs
    P2DIR |= IGNITOR_LED | PILOT_VALVE;
    P2OUT &= ~(IGNITOR_LED | PILOT_VALVE);

    P6DIR |= STATUS_RED | STATUS_GREEN | STATUS_BLUE;
    P6OUT &= ~(STATUS_RED | STATUS_GREEN | STATUS_BLUE);

    // Servo output pin
    P5DIR |= SERVO_PWM;
    P5SEL0 |= SERVO_PWM;
    P5SEL1 &= ~SERVO_PWM;
}

void init_ADC()
{
    // Configure P1.3 (A3), P1.4 (A4), and P1.5 (A5) as analog inputs
    P1SEL0 |= BIT3 | BIT4 | BIT5;
    P1SEL1 |= BIT3 | BIT4 | BIT5;

    ADCCTL0 = ADCSHT_2 | ADCON;  // ADC on, sample-and-hold time
    ADCCTL1 = ADCSHP;           // Use sampling timer
    ADCCTL2 = ADCRES;           // 10-bit resolution
}

unsigned int read_ADC(unsigned int channel)
{
    ADCCTL0 &= ~ADCENC;                     // Disable ADC before changing channel
    ADCMCTL0 = (ADCMCTL0 & ~ADCINCH_15) | channel; // Mask channel bits, then set new channel
    ADCCTL0 |= ADCENC | ADCSC;             // Enable and start conversion
    while (!(ADCIFG & ADCIFG0));           // Wait for conversion to finish
    return ADCMEM0;
}

void init_TimerB_PWM()
{
    TB2CCR0 = 20000 - 1;               // 20ms period (50Hz)
    TB2CCTL1 = OUTMOD_7;               // Reset/Set output mode
    TB2CCR1 = 1000;                    // 1ms pulse = servo min angle
    TB2CTL = TBSSEL__SMCLK | MC__UP | TBCLR; // SMCLK, up mode, clear timer
}

void update_status_LED(unsigned char status)
{
    switch (status)
    {
        // Standby is blue
        case STANDBY:
            P6OUT |= STATUS_BLUE;
            P6OUT &= ~(STATUS_RED | STATUS_GREEN);
            break;
        // Heating is red
        case HEATING:
            P6OUT |= STATUS_RED;
            P6OUT &= ~(STATUS_GREEN | STATUS_BLUE);
            break;
        // Up to temp is green
        case UP_TO_TEMP:
            P6OUT |= STATUS_GREEN;
            P6OUT &= ~(STATUS_RED | STATUS_BLUE);
            break;
    }
}

void turn_everything_off()
{
    P2OUT &= ~(IGNITOR_LED | PILOT_VALVE);
    TB2CCR1 = 1000;    // Close main valve (servo PWM off position)
    call_for_heat_flag = 0;
}

void ignition_sequence()
{
    P2OUT |= PILOT_VALVE;
    
    // Blink LED for ~2 seconds
    int i;
    for (i = 0; i < 10; i++) {   // 10 blinks: 200 ms per blink (100 ms ON, 100 ms OFF)
        P2OUT ^= IGNITOR_LED;        // Toggle LED
        __delay_cycles(100000);      // Delay ~100 ms (assuming 1 MHz clock)
    }

    flame_detected = (unsigned int)thermocouple_to_temperature(read_ADC(THERMOCOUPLE_CH));

    if (flame_detected > 40) {
        TB2CCR1 = 2500; // Adjust pulse width to open main valve
    } else {
        turn_everything_off();
    }
}

// Interrupt for Call-for-Heat falling edge
#pragma vector=PORT1_VECTOR
__interrupt void Port_1_ISR(void)
{
    if (P1IFG & CALL_FOR_HEAT)
    {
        call_for_heat_flag = 1;
        P1IFG &= ~CALL_FOR_HEAT;
    }
}

float thermistor_to_temperature(unsigned int adc_value)
{
    float min_temp = 70.0;  // 50°F minimum
    float max_temp = 125.0; // 150°F maximum

    return min_temp + ((float)adc_value / 1023.0) * (max_temp - min_temp);


    //return adc_value;


    // const float BETA = 4090.0;        // Beta value for your thermistor
    // const float ROOM_TEMP_K = 298.15; // 25°C in Kelvin
    // const float R_FIXED = 10000.0;    //10k Ohm fixed resistor in your divider
    // const float R0 = 10000.0;        // 10k Ohm thermistor at 25°C
    // float resistance;
    // float temperatureK, temperatureC, temperatureF;

    // // Calculate thermistor resistance
    // resistance = R_FIXED * ((1023.0 / adc_value) - 1.0);

    // // Calculate temperature in Kelvin
    // temperatureK = 1.0 / ( (1.0 / ROOM_TEMP_K) + (1.0 / BETA) * log(resistance / R0) );

    // // Kelvin to Celsius
    // temperatureC = temperatureK - 273.15;

    // // Celsius to Fahrenheit
    // temperatureF = (temperatureC * 9.0 / 5.0) + 32.0;

    // return temperatureF1;
}

float setpoint_to_temperature(unsigned int adc_value)
{
    float min_temp = 50.0;  // 50°F minimum
    float max_temp = 150.0; // 150°F maximum

    return min_temp + ((float)adc_value / 1023.0) * (max_temp - min_temp);
}

float thermocouple_to_temperature(unsigned int adc_value)
{
    float voltage = (adc_value / 1023.0) * 3.3;  // in volts

    // If gain is known (e.g., 100x) and thermocouple is Type K (~41µV/°C)
    float gain = 10.0;
    float sensitivity = 41e-6; // V/°C

    float temp_C = (voltage / gain) / sensitivity;

    // Convert to Fahrenheit
    float temp_F = temp_C * 9.0 / 5.0 + 32.0;

    return temp_F;
}