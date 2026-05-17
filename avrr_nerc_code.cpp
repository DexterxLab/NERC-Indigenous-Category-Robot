#include <avr/io.h>          // AVR register definitions
#include <stdint.h>          // Standard integer types
#include <avr/interrupt.h>   // Interrupt support

#define F_CPU 16000000UL     // CPU frequency = 16 MHz
#include <util/delay.h>      // Delay functions

/* ================== JUNCTION PINS ================== */

#define JunctionPin_L PL3    // Left junction sensor (Port L bit 3)
#define JunctionPin_R PL1    // Right junction sensor (Port L bit 1)

void init_JunctionPin(){
    DDRL &= ~((1 << JunctionPin_L) | (1 << JunctionPin_R)); // Set both pins as input
}

/* ================== TIMER 2 (1ms tick) ================== */

void init_Timer_2(){
    TCCR2A = (1<<WGM21);                       // CTC mode
    TCCR2B = (1<<CS22) | (1<<CS21) | (1<<CS20); // Prescaler 1024
    OCR2A = 156;                               // Compare value for ~1ms
    TIMSK2 = (1<<OCIE2A);                      // Enable compare interrupt
    sei();                                     // Enable global interrupts
}

/* ================== LEFT MOTOR (Timer1) ================== */

#define LeftMotor_PWM_L PB5    // PWM forward
#define LeftMotor_PWM_R PB6    // PWM reverse

void init_Left_Motor(){
    DDRB |= (1<<LeftMotor_PWM_L) | (1<<LeftMotor_PWM_R); // Set as outputs

    TCCR1A |= (1<<WGM11) | (1<<COM1A1);   // Fast PWM, non-inverting
    TCCR1B |= (1<<WGM12) | (1<<WGM13) | (1<<CS11); // Mode 14, prescaler 8

    ICR1 = 1999;     // TOP value → 1kHz PWM
    OCR1A = 0;       // Forward duty
    OCR1B = 0;       // Reverse duty
}

uint8_t BASE_SPEED = 100;   // Default motor speed

void setLeftMotor(int16_t speed){
    if (speed > 255) speed = 255;     // Clamp max
    if (speed < -255) speed = -255;   // Clamp min

    if (speed > 0){
        uint16_t shifted_speed = (speed * 1999L) / 255; // Scale to ICR1
        OCR1A = shifted_speed;   // Forward PWM
        OCR1B = 0;               // Reverse off
    }
    else if (speed < 0){
        uint16_t shifted_speed = (-speed * 1999L) / 255;
        OCR1B = shifted_speed;   // Reverse PWM
        OCR1A = 0;               // Forward off
    }
    else{
        OCR1A = 0;               // Stop
        OCR1B = 0;
    }
}

void setLeftMotorBase(){
    setLeftMotor(BASE_SPEED);    // Run at base speed
}

/* ================== RIGHT MOTOR (Timer3) ================== */

#define RightMotor_PWM_L PE3
#define RightMotor_PWM_R PE4

void init_Right_Motor(){
    DDRE |= (1<<RightMotor_PWM_L) | (1<<RightMotor_PWM_R); // Outputs

    TCCR3A |= (1<<WGM31) | (1<<COM3A1);   // Fast PWM
    TCCR3B |= (1<<WGM32) | (1<<WGM33) | (1<<CS31); // Mode 14, prescaler 8

    ICR3 = 1999;     // 1kHz
    OCR3A = 0;
    OCR3B = 0;
}

void setRightMotor(int16_t speed){
    if (speed > 255) speed = 255;
    if (speed < -255) speed = -255;

    if (speed > 0){
        uint16_t shifted_speed = (speed * 1999L) / 255;
        OCR3A = shifted_speed;   // Forward
        OCR3B = 0;
    }
    else if (speed < 0){
        uint16_t shifted_speed = (-speed * 1999L) / 255;
        OCR3B = shifted_speed;   // Reverse
        OCR3A = 0;
    }
    else{
        OCR3A = 0;
        OCR3B = 0;
    }
}

void setRightMotorBase(){
    setRightMotor(BASE_SPEED);
}

/* ================== TIMER2 COUNTER ================== */

volatile uint32_t timer_2_Count = 0; // Millisecond counter

ISR(TIMER2_COMPA_vect){
    timer_2_Count++;   // Increment every compare match
}

/* ================== ADC (QTR Sensors) ================== */

#define SENSOR_COUNT 8

uint16_t sensor_raw[SENSOR_COUNT];
uint16_t sensor_min[SENSOR_COUNT];
uint16_t sensor_max[SENSOR_COUNT];
uint16_t sensor_calibrated[SENSOR_COUNT];

void init_ADC_pins_QTR(){
    DDRF = 0;                        // Port F as input
    ADMUX |= (1<<REFS0);              // AVcc reference
    ADCSRA |= (1<<ADEN) |             // Enable ADC
              (1<<ADPS0) |
              (1<<ADPS1) |
              (1<<ADPS2);             // Prescaler 128 (125kHz ADC clock)
}

uint16_t adc_read_one(uint8_t channel){
    ADMUX = (ADMUX & 0xF0) | (channel & 0x0F); // Select channel
    ADCSRA |= (1<<ADSC);                       // Start conversion
    while (ADCSRA & (1<<ADSC));                // Wait complete
    return ADC;                                // Return result
}

void adc_read_all(){
    for (uint8_t i = 0; i < SENSOR_COUNT; i++){
        sensor_raw[i] = adc_read_one(i);  // Read each channel
    }
}

/* ================== CALIBRATION ================== */

void calibration_init(){
    for (uint8_t i = 0; i < SENSOR_COUNT; i++){
        sensor_min[i] = 1023;  // Set high initial min
        sensor_max[i] = 0;     // Set low initial max
    }
}

void calibrate_sensors(){
    adc_read_all();            // Read sensors

    for (uint8_t i = 0; i < SENSOR_COUNT; i++){
        if (sensor_raw[i] < sensor_min[i])
            sensor_min[i] = sensor_raw[i];   // Update min

        if (sensor_raw[i] > sensor_max[i])
            sensor_max[i] = sensor_raw[i];   // Update max
    }
}

/* ================== NORMALIZED SENSOR READ ================== */

void read_calibrated_sensors(){
    adc_read_all();

    for (uint8_t i = 0; i < SENSOR_COUNT; i++){
        if (sensor_max[i] != sensor_min[i]){
            uint32_t numerator =
                (uint32_t)(sensor_raw[i] - sensor_min[i]) * 1000;
            uint16_t denominator =
                sensor_max[i] - sensor_min[i];

            sensor_calibrated[i] = numerator / denominator; // Scale 0–1000
        }
        else{
            sensor_calibrated[i] = 0;
        }

        if (sensor_calibrated[i] > 1000)
            sensor_calibrated[i] = 1000; // Clamp
    }
}

/* ================== LINE POSITION ================== */

#define REFERENCE 3500   // Center reference

int read_line_position(void){
    read_calibrated_sensors();

    uint32_t weighted_sum = 0;
    uint32_t total_sum = 0;

    for (uint8_t i = 0; i < SENSOR_COUNT; i++){
        uint16_t weight = i * 1000;   // Position weight
        weighted_sum += (uint32_t)sensor_calibrated[i] * weight;
        total_sum += sensor_calibrated[i];
    }

    if (total_sum == 0)
        return REFERENCE;   // Line lost

    return weighted_sum / total_sum; // Weighted average
}

/* ================== PD CONTROL ================== */

#define KP 4
#define KD 1
#define MAX_SPEED 130

int16_t last_error = 0;

void follow_line(){
    int16_t position = read_line_position();
    int16_t error = position - REFERENCE;

    int16_t correction =
        (error * KP)/100 +
        ((error - last_error) * KD)/100;

    last_error = error;

    int16_t left_speed = BASE_SPEED - correction;
    int16_t right_speed = BASE_SPEED + correction;

    // Clamp speeds
    if (left_speed > MAX_SPEED) left_speed = MAX_SPEED;
    if (left_speed < -MAX_SPEED) left_speed = -MAX_SPEED;
    if (right_speed > MAX_SPEED) right_speed = MAX_SPEED;
    if (right_speed < -MAX_SPEED) right_speed = -MAX_SPEED;

    setLeftMotor(left_speed);
    setRightMotor(right_speed);
}

/* ================== MAIN ================== */

int main(void){

    init_JunctionPin();    // Configure junction inputs
    init_Timer_2();        // Start 1ms timer
    init_Left_Motor();     // Setup left PWM
    init_Right_Motor();    // Setup right PWM
    init_ADC_pins_QTR();   // Setup ADC

    calibration_init();    // Reset calibration

    for (uint16_t i = 0; i < 100; i++){
        calibrate_sensors();  // Collect calibration data
        _delay_ms(10);
    }

    while(1){
        setLeftMotor(100);   // Test run forward
        setRightMotor(100);
        checkJunction();     // Monitor junction sensor
    }
}




uint8_t turn_check = 0;
uint8_t flag_check = 0;
switch (currentState) {
	case FOLLOW_LINE:
	followLine();
	if (junctionCount == 3 && reset==0 && flag_check == 0) {
		turn_check = 1;
		currentState = TURNING;
	} 
	else if (junctionCount == 3 && reset==1 && flag_check == 0) {
		check_Blue=1;
		currentState = CHECK;
	}
	else if (junctionCount == 1 && reset == 1 && flag_check == 2){
		turn_check = 2;
		currentState = TURNING;
	}
	else if (junctionCount == 1 && reset == 2 && flag_check == 2){
		check_Blue = 2;
		currentState = CHECK;
	}
	else if (junctionCount == 1 && reset == 2 && flag_check == 4){
		turn_check = 3;
		currentState = TURNING;
	}
	else if (junctionCount == 1 && reset == 3 && flag_check == 4){
		check_Blue = 3;
		currentState = CHECK;
	}
	else if (junctionCount == 1 && reset == 3 && flag_check == 6){
		turn_check = 4;
		currentState = TURNING;
	}
	else if (junctionCount == 1 && reset == 4 && flag_check == 6){
		check_Blue = 4;
		currentState = CHECK;
	}

	else if(junctionCount == 1 && reset==1 && flag_check == 3){
		junctionCount = 0;
		turn_check = 20;
		angry_Hassan= 2;
		currentState = TURNING;
	}
	else if (junctionCount == 3 && reset==1 && flag_check == 3){
		turn_check = 21;
		junctionCount = 0;
		currentState = TURNING;
	}

	else if (junctionCount == 2 && reset==2 && flag_check == 3){
		turn_check = 24;
		junctionCount = 0;
		currentState = TURNING;
	}


	if (junctionCount == 1 && check_1_reset == 0){
		turnLeft90();
		junctionCount = 0;
		check_1_reset = 1;
	}
	if (junctionCount == 1 && check_1_reset == 1){
		turnRight90();
		junctionCount = 0;
		check_1_reset = 2;
		junctionCount=0;
		angry_Hassan=2;	
	}
	if (junctionCount == 1 && check_1_reset == 2){
		junctionCount = 0;
		reset=17;
		junctionCount=0;
		angry_Hassan=3;
	}

	else if ((reset== 18 || reset == 16 ||reset == 17 ||reset == 20)) {
		currentState=FOLLOW_LINE;
		reset = 2;
		junctionCount=0;
		angry_Hassan = 2;
	}

	else if (junctionCount == 3 && reset==2) {
		currentState=STOPPED;
		if (stopped!=3){
			stopped=2;
		}
		angry_Hassan=2;
	}
	else if (junctionCount == 1 && reset==3) {
		currentState = TURNING;
		angry_Hassan=2;
	}
	else if (junctionCount == 2 && reset==4)
	{
		currentState=TURNING;
	}
	else if (junctionCount == 2 && reset==5)
	{
	currentState=TURNING;}
	else if (junctionCount == 1 && reset==6)
	{
		currentState=STOPPED;
	}
	else if (junctionCount == 2 && reset==15 ){
		currentState=TURNING;
	}
	break;

	case TURNING:
	if (turn_check == 1) {
		turnLeft90();
		junctionCount=0;
		reset=1;
	}
	else if (turn_check == 2){
		turnLeft90();
		junctionCount=0;
		reset=2;
	}
	else if (turn_check == 3){
		turnLeft90();
		junctionCount=0;
		reset=3;
	}
	else if (turn_check == 4){
		turnLeft90();
		junctionCount=0;
		reset=4;
	}

	//for check 1 positive condition
	else if(turn_check == 20){
		turnLeft90();
		junctionCount=0;
		reset=4;
	}
	else if (turn_check == 21){
		turnRight90();
		junctionCount=0;
		reset=20;
	}

    //for check 2 positive
	else if (turn_check = 24;){
		turnRight90();
		junctionCount = 0;
		reset=16;
		angry_Hassan= 3;
	}
	else if (junctionCount == 1 && reset==3) {
		turnRight90();
		reset=4;
		junctionCount=0;
		BASE_SPEED=160;
		MAX_SPEED=160;
		angry_Hassan= 2;
	}
	else if (junctionCount == 1 && check_reset == 1){
		turnLeft90();
	}
	else if (junctionCount == 2 && reset==4){
		turnl(50);
		unsigned long muji_3=millis();
		while(millis()-muji_3<1250){
			setLeftMotor(255);
			setRightMotor(255);
		}
		BASE_SPEED=120;
		MAX_SPEED=120;
		angry_Hassan=3;
		junctionCount=0;
		unsigned long muji_45=millis();
		while(!(junctionCount==1||(millis()-muji_45<400))){
			moveBackward(130);
		}
		junctionCount=0;
		turnR(520);
		followLine();
		unsigned long chakka=millis();
		while(!(junctionCount==2||(millis()-chakka<4000))){
			followLine();
			checkJunction();
		}
		unsigned long chakka1=millis();
		while((millis()-chakka1<1800)){
			followLine();
		}
		unsigned long muji_4=millis();
		stopRobot(1000);
		unsigned long tamey99=millis();
		while(millis()-tamey99<1600){
			moveBackward(120);
		}
		stopRobot(10000);
		//turnl(460);
		reset=6;
		junctionCount=0;
	}
	else if (junctionCount == 1 && reset==5){
		stopRobot(1000);
		reset=6;
		junctionCount=0;
	}
	else if (junctionCount == 2 && reset==15){
		turnRight90();
		junctionCount=0;
		reset = 16;
	}
	
	currentState = FOLLOW_LINE;
	break;

	case CHECK:
		if (check_Blue ==1){
			stopRobot(500);
			if (isBlue()){
				unsigned long huzaifa=millis();
				while(millis()-huzaifa<70){
					followLine();
				}
				turnl(40);
				unsigned long tame564=millis();
				while(millis()-tame564<140){
					moveBackward(130);
				}
				turnLeft90();
				stopRobot(1500);
				unsigned long tame78=millis();
				while(millis()-tame78<130){
					moveBackward(100);
				}
				turnRight90();
				junctionCount = 0;
			}
			else{
				junctionCount=0;
				flag_check = 2;
				}
		}
		else if (check_Blue == 2){
			stopRobot(500);
			if (isBlue()){
				unsigned long tame=millis();
				while(millis()-tame<77){
					followLine();
				}
				turnLeft90();
				stopRobot(1500);
				unsigned long tame78=millis();
				while(millis()-tame78<130){
					moveBackward(100);
				}
				turnRight90();
				junctionCount=0;
				angry_Hassan=2;

			}
			else {
				junctionCount=0;
				flag_check = 4;
				}
			}
		}
		else if (check_Blue == 3){
			stopRobot(500);
			if (isBlue()){
				unsigned long huzaifa1=millis();
				while(millis()-huzaifa1<70){
					followLine();
				}
				turnLeft90();
				stopRobot(1500);
				unsigned long kimi6=millis();
				while(millis()-kimi6<100){
					moveBackward(70);
				}
				turnLeft90();
				delay(100);
				angry_Hassan= 2;
				junctionCount=0;
				angry_Hassan=2;
			}
			else {
				junctionCount=0;
				angry_Hassan = 2;
			}
		}
		if (check_Blue == 4){
			//turnl(300);
			angry_Hassan= 0;
			stopRobot(500);
			if (isBlue() || !(isBlue())){
				unsigned long tame=millis();
				while(millis()-tame<70){
					//moveBackward(100);
					followLine();
				}
				turnLeft90();
				stopRobot(1500);
				unsigned long tame78=millis();
				while(millis()-tame78<100){
					moveBackward(70);
				}
				//turnRight90();
				//delay(100);
				//turnRight180();
				//delay(100);
				turnLeft90();
				reset=18;
				junctionCount=0;
			}
			//turnRight90();
			check_reset = 1;
			junctionCount =0;
			angry_Hassan=0;
			while (1){
				followLine();
				checkJunction();
				if (junctionCount == 2 && check_reset == 1) {
					turnRight90();
					check_reset = 2;
					junctionCount =0;
				}
				if (junctionCount == 2 && check_reset == 2) {
					reset =18;
					junctionCount =0;
					angry_Hassan= 0;
					break;
				}
			}
			break;
		}
	}
	currentState = FOLLOW_LINE;
	break;

	case STOPPED:

	if(stopped==2){
		unsigned long tame2=millis();
		while(millis()-tame2<200){
			followLine();
		}
		unsigned long tame3=millis();
		while(millis()-tame3<400){
			moveBackward(50);
		}
		turnLeft90();
		stopRobot(1500);
		turnRight90();

		reset = 3;
		junctionCount=0;
		angry_Hassan=3;

		unsigned long delay_time = millis();
		while (millis() - delay_time < 270){
			followLine();
		}
		unsigned long clearStart = millis();
		while(millis() - clearStart < 250){
			setLeftMotor(BASE_SPEED);
			setRightMotor(BASE_SPEED);
		}
		junctionCount=0;
		angry_Hassan= 0;

		while(1){

			followLine();
			checkJunction();
			if (junctionCount == 1 && reset==3) {
				turnRight90();
				reset=4;
				junctionCount=0;
				BASE_SPEED=160;
				MAX_SPEED=160;
				unsigned long karori=millis();
				while(millis()-karori<500){
					followLine();
				}
				junctionCount=0;
				angry_Hassan= 2;
			}

			else if (junctionCount == 2 && reset==4){
				BASE_SPEED=255;
				turnl(45);
				junctionCount=0;
				unsigned long muji_3=millis();
				while((junctionCount<3) && (millis()-muji_3<1240)){
					checkJunction();
					setLeftMotor(255);
					setRightMotor(255);
				}
				//
				BASE_SPEED=170;
				//turnl(130);
				stopRobot(100);
				unsigned long chakka1=millis();
				while((millis()-chakka1<200)){
					moveBackward(130);
				}
				turnR(134);
				unsigned long chakka23=millis();
				while((millis()-chakka23<370)){
					followLine();
				}
				stopRobot(300);
				//followLine();
				//turnR(40);
				BASE_SPEED=120;
				MAX_SPEED=120;
				angry_Hassan=3;
				junctionCount=0;
				//turnRight90();
				unsigned long muji_45=millis();
				while(millis()-muji_45<1400){
					moveBackward(130);
				}
				junctionCount=0;
				stopRobot(1000000);

		}
		currentState = FOLLOW_LINE;
	}

	currentState = FOLLOW_LINE;
	break;
}