/*
 * ============================================================
 *  LINE FOLLOWER ROBOT — Full AVR C Port
 *  Target: ATmega2560 @ 16 MHz
 *  Translated from Arduino source by user request.
 * ============================================================
 *
 *  PIN MAPPING (matches Arduino Mega header → AVR port/pin)
 *  ─────────────────────────────────────────────────────────
 *  Junction Right  → Digital 22  → PA0
 *  Junction Left   → Digital 53  → PB0
 *
 *  Left  Motor L_EN  → Digital 2   → PE4  (must be HIGH to enable BTS7960)
 *  Left  Motor PWM_L → Digital 5   → PE3  (OC3A)
 *  Left  Motor PWM_R → Digital 6   → PH3  (OC4A)
 *
 *  Right Motor R_EN  → Digital 12  → PB6
 *  Right Motor PWM_L → Digital 9   → PH6  (OC2B)
 *  Right Motor PWM_R → Digital 10  → PB4  (OC2A)
 *
 *  QTR Sensors       → A0–A7      → PF0–PF7 (ADC0–ADC7)
 *
 *  TCS3200 S1        → Digital 4   → PG5
 *  TCS3200 S2        → Digital 7   → PH4
 *  TCS3200 S3        → Digital 8   → PH5
 *  TCS3200 OUT       → Digital 3   → PE5  (ICP3 / INT5)
 *
 *  Servo X           → Digital 44  → PL5  (OC5C)
 *  Servo Y           → Digital 45  → PL4  (OC5B)
 *
 *  Solenoid          → Digital 41  → PG0
 *
 *  Timer2 (TIMER0 on AVR) → used for millis() tick (1 ms via CTC on Timer0)
 * ============================================================
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <stdbool.h>

#define F_CPU 16000000UL
#include <util/delay.h>

/* ============================================================
 *  MILLISECOND TIMER  (Timer0 CTC, 1 ms tick)
 * ============================================================ */

volatile uint32_t _millis_count = 0;

void init_millis_timer(void) {
    /* CTC mode, prescaler 64 → 16e6/64 = 250 000 Hz
       OCR0A = 249 → interrupt every 1 ms                    */
    TCCR0A = (1 << WGM01);
    TCCR0B = (1 << CS01) | (1 << CS00);   /* prescaler 64 */
    OCR0A  = 249;
    TIMSK0 = (1 << OCIE0A);
    sei();
}

ISR(TIMER0_COMPA_vect) {
    _millis_count++;
}

static inline uint32_t millis(void) {
    uint32_t val;
    uint8_t sreg = SREG;
    cli();
    val = _millis_count;
    SREG = sreg;
    return val;
}

static inline void delay_ms(uint32_t ms) {
    uint32_t start = millis();
    while ((millis() - start) < ms);
}

/* ============================================================
 *  JUNCTION PINS
 *  Right → Digital 22 → PA0
 *  Left  → Digital 53 → PB0
 * ============================================================ */

#define JUNCTION_RIGHT_PORT  PINA
#define JUNCTION_RIGHT_DDR   DDRA
#define JUNCTION_RIGHT_PIN   PA0

#define JUNCTION_LEFT_PORT   PINB
#define JUNCTION_LEFT_DDR    DDRB
#define JUNCTION_LEFT_PIN    PB0

void init_junction_pins(void) {
    JUNCTION_RIGHT_DDR &= ~(1 << JUNCTION_RIGHT_PIN);  /* input */
    JUNCTION_LEFT_DDR  &= ~(1 << JUNCTION_LEFT_PIN);   /* input */
    /* Enable internal pull-ups (matches INPUT_PULLUP in Arduino) */
    PORTA |= (1 << JUNCTION_RIGHT_PIN);
    PORTB |= (1 << JUNCTION_LEFT_PIN);
}

static inline bool read_junction_right(void) {
    return (JUNCTION_RIGHT_PORT >> JUNCTION_RIGHT_PIN) & 1;
}
static inline bool read_junction_left(void) {
    return (JUNCTION_LEFT_PORT >> JUNCTION_LEFT_PIN) & 1;
}

/* ============================================================
 *  MOTOR DRIVER  (BTS7960 — two half-bridge channels each)
 *
 *  Left Motor:
 *    L_EN   → Digital 2  → PE4  (direction/enable)
 *    PWM_L  → Digital 5  → PE3  (OC3A — Timer3 channel A)
 *    PWM_R  → Digital 6  → PH3  (OC4A — Timer4 channel A)
 *
 *  Right Motor:
 *    R_EN   → Digital 12 → PB6
 *    PWM_L  → Digital 9  → PH6  (OC2B — Timer2 channel B)
 *    PWM_R  → Digital 10 → PB4  (OC2A — Timer2 channel A)
 *
 *  PWM frequency target: ~1 kHz
 * ============================================================ */

/* --- Left Motor Enable: PE4 (Digital 2) --- */
#define LM_EN_DDR   DDRE
#define LM_EN_PORT  PORTE
#define LM_EN_PIN   PE4

/* --- Left Motor PWM_L: PE3 / OC3A (Digital 5) --- */
/* --- Left Motor PWM_R: PH3 / OC4A (Digital 6) --- */
#define LM_PWML_DDR  DDRE
#define LM_PWML_PORT PORTE
#define LM_PWML_PIN  PE3

#define LM_PWMR_DDR  DDRH
#define LM_PWMR_PORT PORTH
#define LM_PWMR_PIN  PH3

/* --- Right Motor Enable: PB6 (Digital 12) --- */
#define RM_EN_DDR   DDRB
#define RM_EN_PORT  PORTB
#define RM_EN_PIN   PB6

/* --- Right Motor PWM_L: PH6 / OC2B (Digital 9) --- */
/* --- Right Motor PWM_R: PB4 / OC2A (Digital 10) --- */
#define RM_PWML_DDR  DDRH
#define RM_PWML_PORT PORTH
#define RM_PWML_PIN  PH6

#define RM_PWMR_DDR  DDRB
#define RM_PWMR_PORT PORTB
#define RM_PWMR_PIN  PB4

/*
 *  Timer3 → Left Motor PWM_L  (OC3A, Phase-correct PWM, TOP=ICR3)
 *  Timer4 → Left Motor PWM_R  (OC4A, Phase-correct PWM, TOP=ICR4)
 *  Timer2 → Right Motor (OC2A / OC2B, Fast PWM 8-bit)
 *
 *  For ~1 kHz with 8-bit resolution on 16 MHz:
 *    Phase-correct PWM, prescaler 8, TOP=999 →
 *    f = 16e6 / (2 * 8 * 1000) = 1000 Hz  ✓
 *
 *  Timer2 is 8-bit so prescaler 64 gives:
 *    Fast PWM, prescaler 64, TOP=255 →
 *    f = 16e6 / (64 * 256) ≈ 977 Hz  ✓
 */

void init_motors(void) {
    /* Enable outputs */
    LM_EN_DDR   |= (1 << LM_EN_PIN);
    LM_PWML_DDR |= (1 << LM_PWML_PIN);
    LM_PWMR_DDR |= (1 << LM_PWMR_PIN);
    RM_EN_DDR   |= (1 << RM_EN_PIN);
    RM_PWML_DDR |= (1 << RM_PWML_PIN);
    RM_PWMR_DDR |= (1 << RM_PWMR_PIN);

    /* Enable both motor drivers */
    LM_EN_PORT  |= (1 << LM_EN_PIN);
    RM_EN_PORT  |= (1 << RM_EN_PIN);

    /* --- Timer3: Left PWM_L (OC3A on PE3) ---
       Phase-correct PWM mode 10, TOP=ICR3, prescaler 8           */
    TCCR3A = (1 << WGM31) | (1 << COM3A1);          /* non-inverting */
    TCCR3B = (1 << WGM33) | (1 << CS31);            /* prescaler 8   */
    ICR3   = 999;
    OCR3A  = 0;

    /* --- Timer4: Left PWM_R (OC4A on PH3) ---  */
    TCCR4A = (1 << WGM41) | (1 << COM4A1);
    TCCR4B = (1 << WGM43) | (1 << CS41);
    ICR4   = 999;
    OCR4A  = 0;

    /* --- Timer2: Right Motor (OC2A=PB4, OC2B=PH6) ---
       Fast PWM 8-bit, prescaler 64                                */
    TCCR2A = (1 << WGM21) | (1 << WGM20) |          /* Fast PWM      */
             (1 << COM2A1) | (1 << COM2B1);          /* non-inverting */
    TCCR2B = (1 << CS22);                            /* prescaler 64  */
    OCR2A  = 0;
    OCR2B  = 0;
}

/* ---- helper: scale 0-255 speed to Timer3/4 TOP=999 range ---- */
static inline uint16_t scale_to_1000(uint8_t s) {
    return (uint16_t)((uint32_t)s * 999 / 255);
}

int16_t BASE_SPEED = 130;
int16_t MAX_SPEED  = 140;
int16_t TURN_SPEED = 100;

void setLeftMotor(int16_t speed) {
    if (speed >  255) speed =  255;
    if (speed < -255) speed = -255;
    if (speed > 0) {
        OCR3A = scale_to_1000((uint8_t)speed);   /* PWM_L forward */
        OCR4A = 0;
    } else if (speed < 0) {
        OCR4A = scale_to_1000((uint8_t)(-speed));/* PWM_R reverse */
        OCR3A = 0;
    } else {
        OCR3A = 0;
        OCR4A = 0;
    }
}

void setRightMotor(int16_t speed) {
    if (speed >  255) speed =  255;
    if (speed < -255) speed = -255;
    if (speed > 0) {
        OCR2B = (uint8_t)speed;   /* PWM_L forward */
        OCR2A = 0;
    } else if (speed < 0) {
        OCR2A = (uint8_t)(-speed);/* PWM_R reverse */
        OCR2B = 0;
    } else {
        OCR2A = 0;
        OCR2B = 0;
    }
}

void moveBackward(int16_t speed) {
    if (speed < 0) speed = -speed;
    setLeftMotor(-speed);
    setRightMotor(-speed);
}

void stopRobot(uint32_t durationMs) {
    uint32_t start = millis();
    while ((millis() - start) < durationMs) {
        setLeftMotor(0);
        setRightMotor(0);
    }
}

/* ============================================================
 *  ADC  (QTR-8A — 8 analog sensors on A0–A7 = PF0–PF7)
 * ============================================================ */

#define SENSOR_COUNT 8

uint16_t sensorValues[SENSOR_COUNT];     /* calibrated 0–1000 */
uint16_t sensor_min[SENSOR_COUNT];
uint16_t sensor_max[SENSOR_COUNT];
uint16_t sensor_raw[SENSOR_COUNT];

void init_ADC(void) {
    DDRF  = 0x00;                            /* Port F all inputs */
    ADMUX  = (1 << REFS0);                   /* AVcc reference */
    ADCSRA = (1 << ADEN) |
             (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); /* prescaler 128 */
}

uint16_t adc_read(uint8_t ch) {
    ADMUX  = (ADMUX & 0xF0) | (ch & 0x0F);
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    return ADC;
}

void adc_read_all(void) {
    for (uint8_t i = 0; i < SENSOR_COUNT; i++)
        sensor_raw[i] = adc_read(i);
}

/* ============================================================
 *  CALIBRATION
 * ============================================================ */

void calibration_init(void) {
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        sensor_min[i] = 1023;
        sensor_max[i] = 0;
    }
}

void calibrate_sensors(void) {
    adc_read_all();
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        if (sensor_raw[i] < sensor_min[i]) sensor_min[i] = sensor_raw[i];
        if (sensor_raw[i] > sensor_max[i]) sensor_max[i] = sensor_raw[i];
    }
}

/* ============================================================
 *  CALIBRATED READ + WEIGHTED LINE POSITION
 *  Returns value 0–7000 (like QTRSensors::readLineBlack())
 * ============================================================ */

#define REFERENCE  3500
#define LINE_THRESHOLD    700   /* sensor value = "sees line"  */
#define WHITE_THRESHOLD   400   /* sensor value = "white/no line" */
#define CENTER_THRESHOLD  750

uint16_t read_line_black(void) {
    adc_read_all();
    /* Normalize raw → 0–1000 */
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        if (sensor_max[i] != sensor_min[i]) {
            uint32_t num = (uint32_t)(sensor_raw[i] - sensor_min[i]) * 1000;
            sensorValues[i] = (uint16_t)(num / (sensor_max[i] - sensor_min[i]));
        } else {
            sensorValues[i] = 0;
        }
        if (sensorValues[i] > 1000) sensorValues[i] = 1000;
    }
    /* Weighted average */
    uint32_t weighted = 0, total = 0;
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        weighted += (uint32_t)sensorValues[i] * (i * 1000);
        total    += sensorValues[i];
    }
    if (total == 0) return REFERENCE;
    return (uint16_t)(weighted / total);
}

static inline bool lineDetectedCenter(void) {
    return (sensorValues[5] > 800 && sensorValues[4] > 800);
}

bool allSensorsWhite(void) {
    adc_read_all();
    for (uint8_t i = 7; i < SENSOR_COUNT; i++)
        if (sensorValues[i] > WHITE_THRESHOLD) return false;
    return true;
}

/* ============================================================
 *  PD LINE FOLLOWING
 * ============================================================ */

double kp = 0.04;
double kd = 0.01;
double lastError = 0.0;

void followLine(void) {
    int16_t position  = (int16_t)read_line_black();
    int16_t error     = position - REFERENCE;
    int16_t correction = (int16_t)(kp * error + kd * (error - lastError));
    lastError = error;

    int16_t leftSpeed  = BASE_SPEED - correction;
    int16_t rightSpeed = BASE_SPEED + correction;

    if (leftSpeed  >  MAX_SPEED) leftSpeed  =  MAX_SPEED;
    if (leftSpeed  < -MAX_SPEED) leftSpeed  = -MAX_SPEED;
    if (rightSpeed >  MAX_SPEED) rightSpeed =  MAX_SPEED;
    if (rightSpeed < -MAX_SPEED) rightSpeed = -MAX_SPEED;

    setLeftMotor(leftSpeed);
    setRightMotor(rightSpeed);
}

/* ============================================================
 *  JUNCTION DETECTION  (debounced, state-machine driven)
 * ============================================================ */

volatile int  junctionCount        = 0;
volatile int  State                = 0;   /* sensor-select mode */
unsigned long lastJunctionTime     = 0;
bool          lastJunctionState    = false;
#define       JUNCTION_DEBOUNCE_MS 500

void checkJunction(void) {
    bool currentState = false;
    uint32_t currentTime = millis();

    if      (State == 0) currentState =  read_junction_right();
    else if (State == 2) currentState =  read_junction_left();
    else if (State == 3) currentState =  read_junction_left() && (!read_junction_right());
    else if (State == 4) currentState =  read_junction_left() ||  (!read_junction_right());

    /* Falling-edge detect with debounce */
    if (lastJunctionState == true && currentState == false) {
        if ((currentTime - lastJunctionTime) > JUNCTION_DEBOUNCE_MS) {
            junctionCount++;
            lastJunctionTime = currentTime;
        }
    }
    lastJunctionState = currentState;
}

/* ============================================================
 *  TURNS
 * ============================================================ */

void turnRight90(void) {
    /* Phase 1: clear intersection */
    setLeftMotor(100); setRightMotor(100);
    delay_ms(90);

    /* Phase 2: coarse spin */
    uint32_t t = millis();
    while ((millis() - t) < 100) {
        setLeftMotor(TURN_SPEED);
        setRightMotor(-TURN_SPEED);
    }
    delay_ms(150);

    /* Phase 3: find new line */
    t = millis();
    while (true) {
        read_line_black();
        setLeftMotor(BASE_SPEED);
        setRightMotor(-BASE_SPEED);
        if (sensorValues[0] > 700 && sensorValues[1] > 700) break;
        if ((millis() - t) > 1500) break;
    }

    /* Phase 4: fine align */
    while (true) {
        read_line_black();
        setLeftMotor(90);
        setRightMotor(-90);
        if (lineDetectedCenter()) {
            setLeftMotor(-50); setRightMotor(50);
            delay_ms(30);
            break;
        }
    }
    stopRobot(100);
}

void turnLeft90(void) {
    /* Phase 1: clear intersection */
    setLeftMotor(100); setRightMotor(100);
    delay_ms(70);

    /* Phase 2: coarse spin */
    uint32_t t = millis();
    while ((millis() - t) < 100) {
        setLeftMotor(-TURN_SPEED);
        setRightMotor(TURN_SPEED);
    }
    delay_ms(150);

    /* Phase 3: find new line */
    t = millis();
    while (true) {
        read_line_black();
        setLeftMotor(-TURN_SPEED);
        setRightMotor(TURN_SPEED);
        if (sensorValues[6] > 700 || sensorValues[7] > 700) break;
        if ((millis() - t) > 1500) break;
    }

    /* Phase 4: fine align */
    while (true) {
        read_line_black();
        setLeftMotor(-80);
        setRightMotor(80);
        if (lineDetectedCenter()) {
            setLeftMotor(50); setRightMotor(-50);
            delay_ms(30);
            break;
        }
    }
    stopRobot(100);
}

/* ============================================================
 *  TCS3200 COLOR SENSOR
 *  S1  → Digital 4  → PG5
 *  S2  → Digital 7  → PH4
 *  S3  → Digital 8  → PH5
 *  OUT → Digital 3  → PE5  (read with pulseIn equivalent)
 * ============================================================ */

#define TCS_S1_DDR  DDRG
#define TCS_S1_PORT PORTG
#define TCS_S1_PIN  PG5

#define TCS_S2_DDR  DDRH
#define TCS_S2_PORT PORTH
#define TCS_S2_PIN  PH4

#define TCS_S3_DDR  DDRH
#define TCS_S3_PORT PORTH
#define TCS_S3_PIN  PH5

#define TCS_OUT_DDR  DDRE
#define TCS_OUT_PIN  PE5
#define TCS_OUT_PORT PINE   /* read register */

void init_color_sensor(void) {
    TCS_S1_DDR  |= (1 << TCS_S1_PIN);
    TCS_S2_DDR  |= (1 << TCS_S2_PIN);
    TCS_S3_DDR  |= (1 << TCS_S3_PIN);
    TCS_OUT_DDR &= ~(1 << TCS_OUT_PIN);  /* input */

    /* Set frequency scaling to ~20% (S0=low, S1=high in Arduino code;
       S0 pin was commented out but S1 was set LOW)                    */
    TCS_S1_PORT &= ~(1 << TCS_S1_PIN);  /* S1 LOW */
}

/* pulseIn equivalent: measure LOW pulse width in microseconds.
   Timeout = 10 000 µs (matches Arduino call).                       */
static uint32_t pulseIn_LOW(volatile uint8_t *port, uint8_t pin, uint32_t timeout_us) {
    /* Wait for any current HIGH to end */
    uint32_t start = millis();
    while ( (*port >> pin) & 1 ) {
        if ((millis() - start) * 1000 > timeout_us) return 0;
    }
    /* Wait for LOW to start */
    start = millis();
    while (!( (*port >> pin) & 1 )) {   /* wait for HIGH (wrong) */
        /* actually wait until pin goes LOW */
        if ((millis() - start) * 1000 > timeout_us) return 0;
    }
    /* Measure LOW pulse — use cycle counting for µs resolution */
    uint32_t count = 0;
    /* Wait for pin to go LOW */
    while ( (*port >> pin) & 1 );
    /* Count while LOW */
    while (!( (*port >> pin) & 1 )) count++;
    /* Approximate: each loop ≈ 4 cycles @ 16MHz → 0.25 µs/count */
    return count / 4;
}

uint32_t readBlue(void) {
    TCS_S2_PORT &= ~(1 << TCS_S2_PIN);  /* S2 LOW  */
    TCS_S3_PORT |=  (1 << TCS_S3_PIN);  /* S3 HIGH */
    _delay_us(100);
    return pulseIn_LOW(&TCS_OUT_PORT, TCS_OUT_PIN, 10000);
}

uint32_t readRed(void) {
    TCS_S2_PORT &= ~(1 << TCS_S2_PIN);  /* S2 LOW */
    TCS_S3_PORT &= ~(1 << TCS_S3_PIN);  /* S3 LOW */
    _delay_ms(2);
    return pulseIn_LOW(&TCS_OUT_PORT, TCS_OUT_PIN, 10000);
}

uint32_t readGreen(void) {
    TCS_S2_PORT |= (1 << TCS_S2_PIN);   /* S2 HIGH */
    TCS_S3_PORT |= (1 << TCS_S3_PIN);   /* S3 HIGH */
    _delay_ms(2);
    return pulseIn_LOW(&TCS_OUT_PORT, TCS_OUT_PIN, 10000);
}

bool isBlue(void) {
    uint32_t r = readRed();
    uint32_t b = readBlue();
    /* Blue surface: red pulse noticeably larger than blue */
    return (r > (b * 13 / 10));   /* 1.3× threshold */
}

/* ============================================================
 *  SERVO  (Timer5 — OC5B = Digital 45 = PL4,  OC5C = Digital 44 = PL5)
 *
 *  Standard servo: 1 ms–2 ms pulse in a 20 ms period.
 *  Timer5: Phase-correct PWM, TOP=ICR5, prescaler 8
 *    Period = 2 * prescaler * TOP / F_CPU
 *    20 ms  = 2 * 8 * TOP / 16e6  →  TOP = 20000
 *    1 ms pulse → OCR = 1000
 *    2 ms pulse → OCR = 2000
 *  Angle 0–180 → OCR 1000–2000
 * ============================================================ */

#define SERVO_X_DDR  DDRL
#define SERVO_X_PORT PORTL
#define SERVO_X_PIN  PL5   /* OC5C */

#define SERVO_Y_DDR  DDRL
#define SERVO_Y_PORT PORTL
#define SERVO_Y_PIN  PL4   /* OC5B */

void init_servos(void) {
    SERVO_X_DDR |= (1 << SERVO_X_PIN);
    SERVO_Y_DDR |= (1 << SERVO_Y_PIN);

    /* Timer5: Phase-correct PWM mode 10 (WGM5=1010), TOP=ICR5 */
    TCCR5A = (1 << WGM51)  |
             (1 << COM5B1) |   /* OC5B non-inverting */
             (1 << COM5C1);    /* OC5C non-inverting */
    TCCR5B = (1 << WGM53) | (1 << CS51);   /* prescaler 8 */
    ICR5   = 20000;

    OCR5B  = 1400;   /* servoY initial ~40° */
    OCR5C  = 1400;   /* servoX initial ~40° */
}

static inline uint16_t angle_to_ocr(uint8_t angle) {
    if (angle > 180) angle = 180;
    /* map 0–180 → 1000–2000 */
    return (uint16_t)(1000 + (uint32_t)angle * 1000 / 180);
}

void servoX_write(uint8_t angle) { OCR5C = angle_to_ocr(angle); }
void servoY_write(uint8_t angle) { OCR5B = angle_to_ocr(angle); }

void moveServoXY(uint8_t targetX, uint8_t targetY, uint16_t delayTime) {
    if (targetX > 180) targetX = 180;
    if (targetY > 180) targetY = 180;
    servoY_write(targetY);
    delay_ms(delayTime);
    servoX_write(targetX);
    delay_ms(delayTime);
}

/* ============================================================
 *  SOLENOID   Digital 41 → PG0
 * ============================================================ */

#define SOL_DDR  DDRG
#define SOL_PORT PORTG
#define SOL_PIN  PG0

void init_solenoid(void) {
    SOL_DDR  |= (1 << SOL_PIN);
    SOL_PORT |= (1 << SOL_PIN);   /* HIGH = off (active LOW) */
}

void fireSolenoid(uint16_t pulseTime) {
    SOL_PORT &= ~(1 << SOL_PIN);  /* energize */
    delay_ms(pulseTime);
    SOL_PORT |=  (1 << SOL_PIN);  /* off */
}

/* ============================================================
 *  STATE MACHINE
 * ============================================================ */

typedef enum {
    FOLLOW_LINE,
    TURNING,
    STOPPED,
    CHECK
} RobotState;

RobotState currentState = FOLLOW_LINE;

int  Flag          = 0;
int  reset_var     = 0;   /* renamed from 'reset' to avoid C keyword clash */
int  stopped_flag  = 0;
int  check_Blue    = 0;

/* ============================================================
 *  MAIN
 * ============================================================ */

int main(void) {

    /* --- Peripheral Init --- */
    init_millis_timer();
    init_junction_pins();
    init_motors();
    init_ADC();
    init_color_sensor();
    init_servos();
    init_solenoid();
    sei();

    /* --- QTR Calibration (100 × 10 ms = 1 s) --- */
    calibration_init();
    for (uint16_t i = 0; i < 100; i++) {
        calibrate_sensors();
        delay_ms(10);
    }

    /* --- Servo home position --- */
    servoX_write(40);
    servoY_write(40);

    /* ====================================================
     *  MAIN LOOP
     * ==================================================== */
    while (1) {

        checkJunction();

        switch (currentState) {

        /* -------------------------------------------------- */
        case FOLLOW_LINE:
            followLine();

            if      (junctionCount == 3 && reset_var == 0  && Flag == 0) { currentState = TURNING; }
            else if (junctionCount == 3 && reset_var == 1  && Flag == 0) {
                currentState = CHECK;
                uint32_t t = millis();
                while ((millis() - t) < 100) { setLeftMotor(100); setRightMotor(100); }
                check_Blue = 1;
                State = 2;
            }
            else if (junctionCount == 1 && reset_var == 2  && Flag == 1) { currentState = TURNING; State = 2; }
            else if (junctionCount == 3 && reset_var == 3  && Flag == 1) { currentState = TURNING; /* State==3 (was assignment bug in Arduino) */ State = 3; }
            else if (junctionCount == 1 && reset_var == 2  && Flag == 0) { currentState = TURNING; }
            else if (junctionCount == 1 && reset_var == 3  && Flag == 0) {
                currentState = CHECK;
                uint32_t t1 = millis();
                while ((millis() - t1) < 100) { setLeftMotor(100); setRightMotor(100); }
                State = 2;
            }
            else if (junctionCount == 2 && reset_var == 4  && Flag == 2) { currentState = TURNING; }
            else if (junctionCount == 1 && reset_var == 4  && Flag == 0) { currentState = TURNING; State = 2; }
            else if (junctionCount == 1 && reset_var == 50 && Flag == 0) {
                currentState = CHECK;
                uint32_t t2 = millis();
                while ((millis() - t2) < 100) { setLeftMotor(100); setRightMotor(100); }
            }
            else if (junctionCount == 1 && reset_var == 5  && Flag == 3) { currentState = TURNING; }
            else if (junctionCount == 1 && reset_var == 5  && Flag == 0) { currentState = TURNING; State = 0; }
            else if (junctionCount == 1 && reset_var == 6  && Flag == 0) { currentState = TURNING; State = 0; }
            else if (junctionCount == 2 && reset_var == 7  && Flag == 0) { currentState = TURNING; }
            else if (junctionCount == 2 && reset_var == 8  && Flag == 0) { currentState = TURNING; State = 2; }
            else if (junctionCount == 3 && reset_var == 99)              { currentState = STOPPED; }
            else if (junctionCount == 1 && reset_var == 100)             { currentState = TURNING; }
            else if (junctionCount == 2 && reset_var == 101)             { currentState = TURNING; }
            break;

        /* -------------------------------------------------- */
        case TURNING:
            if      (junctionCount == 3 && reset_var == 0 && Flag == 0) {
                turnLeft90(); junctionCount = 0; reset_var = 1;
            }
            else if (junctionCount == 1 && reset_var == 2 && Flag == 1) {
                turnLeft90(); junctionCount = 0; reset_var = 3;
            }
            else if (junctionCount == 3 && reset_var == 3 && Flag == 1) {
                turnRight90(); junctionCount = 0; reset_var = 99;
            }
            else if (junctionCount == 1 && reset_var == 2 && Flag == 0) {
                turnLeft90(); junctionCount = 0; reset_var = 3;
            }
            else if (junctionCount == 2 && reset_var == 4 && Flag == 2) {
                turnRight90(); junctionCount = 0; reset_var = 99;
            }
            else if (junctionCount == 1 && reset_var == 4 && Flag == 0) {
                turnLeft90(); junctionCount = 0; reset_var = 50;
            }
            else if (junctionCount == 1 && reset_var == 5 && Flag == 3) {
                turnRight90();
                junctionCount = 0;
                while (junctionCount == 0) { followLine(); checkJunction(); }
                junctionCount = 0;
                reset_var = 99;
            }
            else if (junctionCount == 1 && reset_var == 5 && Flag == 0) {
                turnLeft90(); junctionCount = 0; reset_var = 6;
            }
            else if (junctionCount == 1 && reset_var == 6 && Flag == 0) {
                turnLeft90();
                stopRobot(1000);
                moveServoXY(40, 90, 500);
                fireSolenoid(100);
                delay_ms(100);
                moveServoXY(70, 110, 500);
                fireSolenoid(100);
                turnLeft90();
                State = 0;
                junctionCount = 0;
                reset_var = 7;
            }
            else if (junctionCount == 2 && reset_var == 7 && Flag == 0) {
                turnRight90(); junctionCount = 0; reset_var = 8;
            }
            else if (junctionCount == 2 && reset_var == 8 && Flag == 0) {
                junctionCount = 0; reset_var = 99;
            }
            /* Unify */
            else if (junctionCount == 1 && reset_var == 100) {
                turnRight90(); junctionCount = 0; reset_var = 101;
            }
            else if (junctionCount == 2 && reset_var == 101) {
                turnRight90();
                junctionCount = 0;
                while (junctionCount != 1) { followLine(); checkJunction(); }
                turnLeft90();
                BASE_SPEED = 140;
                MAX_SPEED  = 150;
                junctionCount = 0;
                while (junctionCount != 2) { followLine(); checkJunction(); }
                turnRight90();
                followLine();
                delay_ms(100);
                moveBackward(120);
                delay_ms(1500);
                stopRobot(10000);
                reset_var = 102;
                junctionCount = 0;
            }
            currentState = FOLLOW_LINE;
            break;

        /* -------------------------------------------------- */
        case STOPPED:
            if (stopped_flag == 0) {
                turnLeft90();
                stopRobot(1500);
                uint32_t t78 = millis();
                while ((millis() - t78) < 100) moveBackward(80);
                turnRight90();
                stopped_flag = 1;
                junctionCount = 0;
                reset_var = 100;
            }
            currentState = FOLLOW_LINE;
            break;

        /* -------------------------------------------------- */
        case CHECK:
            if (check_Blue == 1) {
                stopRobot(1000);
                if (isBlue()) {
                    Flag = 1;
                    stopRobot(1000);
                    moveServoXY(30, 150, 300);
                    fireSolenoid(100);
                    delay_ms(300);
                    moveServoXY(150, 30, 100);
                    fireSolenoid(100);
                    junctionCount = 0;
                    reset_var = 2;
                } else {
                    check_Blue = 2;
                    junctionCount = 0;
                    reset_var = 2;
                }
            }
            else if (check_Blue == 2) {
                stopRobot(1000);
                if (isBlue()) {
                    Flag = 2;
                    turnLeft90();
                    stopRobot(1000);
                    moveServoXY(40, 90, 500);
                    fireSolenoid(100);
                    delay_ms(100);
                    moveServoXY(70, 110, 500);
                    fireSolenoid(100);
                    turnRight90();
                    junctionCount = 0;
                    reset_var = 4;
                } else {
                    check_Blue = 3;
                    junctionCount = 0;
                    reset_var = 4;
                }
            }
            else if (check_Blue == 3) {
                stopRobot(1000);
                if (isBlue()) {
                    Flag = 3;
                    turnLeft90();
                    stopRobot(1000);
                    moveServoXY(40, 90, 500);
                    fireSolenoid(100);
                    delay_ms(100);
                    moveServoXY(70, 110, 500);
                    fireSolenoid(100);
                    turnRight90();
                    turnRight90();
                    junctionCount = 0;
                    reset_var = 5;
                } else {
                    check_Blue = 4;
                    junctionCount = 0;
                    reset_var = 5;
                }
            }
            currentState = FOLLOW_LINE;
            break;

        } /* end switch */

    } /* end while(1) */
    return 0;
}
