/*
 Name:		Motor_code.ino
 Created:	18-May-18 6:37:53 PM
 Author:	Ian
*/

#define		TC0_SYNC					(*(WoReg*) 0x400800C0)		//the address of the sync register used to force TC0 to synchronise all it's channels
#define		TC2_SYNC					(*(WoReg*) 0x400880C0)		//the address of the sync register used to force TC2 to synchronise all it's channels

#define		TIMER_COUNTER1 				41999026		//clock 1 = MCK/2
#define		FREQ			 			32000		//100KHz PWM signal desired for the motors
#define		LEFT_TC						TC0			// TC number
#define		RIGHT_TC					TC2			// TC number

#define		LEFT_MOTOR_CHAN				0			// TC channel
#define		LEFT_MOTOR_ID				ID_TC0		// Instance ID
#define		LEFT_MOTOR_PIOPIN			25			// Peripheral B channel number digital pin 2
#define		LEFT_MOTOR_RA				(*(WoReg*) 0x40080014)		//the address of the RA register used to control the duty cycle of TC0 CH0

#define		LEFT_MOTOR_COMP_CHAN		1			// TC channel
#define		LEFT_MOTOR_COMP_ID			ID_TC1		// Instance ID
#define		LEFT_MOTOR_COMP_PIOPIN		2			// Peripheral A channel number analogue pin 7
#define		LEFT_MOTOR_COMP_RA			(*(WoReg*) 0x40080054)		//the address of the RA register used to control the duty cycle of TC0 CH1

#define		RIGHT_MOTOR_CHAN			0			// TC channel
#define		RIGHT_MOTOR_ID				ID_TC6		// Instance ID
#define		RIGHT_MOTOR_PIOPIN			25			// Peripheral C channel number digital pin 5
#define		RIGHT_MOTOR_RA				(*(WoReg*) 0x40088014)		//the address of the RA register used to control the duty cycle of TC2 CH0

#define		RIGHT_MOTOR_COMP_CHAN		1			// TC channel
#define		RIGHT_MOTOR_COMP_ID			ID_TC7		// Instance ID
#define		RIGHT_MOTOR_COMP_PIOPIN		28			// Peripheral C channel number digital pin 3
#define		RIGHT_MOTOR_COMP_RA			(*(WoReg*) 0x40088054)		//the address of the RA register used to control the duty cycle of TC2 CH1

#define		PUMP_CHAN					2			// TC channel
#define		PUMP_ID						ID_TC8		// Instance ID
#define		PUMP_PIOPIN					7			// Peripheral D channel number digital pin 11
#define		PUMP_RA						(*(WoReg*) 0x40088094)		//the address of the RA register used to control the duty cycle of TC2 CH2

//Declaring the pins required to read feedback. Different pins are used for forward and reverse
int leftForwardSensorPin = A0;
int leftReverseSensorPin = A1;
int rightForwardSensorPin = A2;
int rightReverseSensorPin = A3;

unsigned int RC_clk = TIMER_COUNTER1 / FREQ;			//This is the value that RC needs to be to create the specified frequency
int setDutyCycle = 50;
int leftFloatingDutyCycle = 50;
int rightFloatingDutyCycle = 50;
int direction = 0;							//0 means forward, 1 means reverse

//parametre tc_num is a pointer to a Tc instance, parametre complementry identifies if it needs to set up a complementry waveform. If < 0 it's complementry,
//if > 0 it's not and if == 0 it doesn't matter i.e. for the pump
void pwmwave(unsigned int duty, int channel, Tc *tc_num, int complementry) {

	//unsigned int  tcclk = TIMER_COUNTER1 / FREQ;      //Calculates the value required in RC to create a signal with the given freq
	double dutyCycle = (double)duty/100;

	if (complementry > 0) {
		//Will clear when it reaches the value in RC and set when it reaches the value in RA
		TC_Configure(tc_num, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK1 | TC_CMR_ACPC_CLEAR | TC_CMR_ACPA_SET);
	}
	else {
		//Will set when it reaches the value in RC and clear when it reaches the value in RA
		TC_Configure(tc_num, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK1 | TC_CMR_ACPC_SET | TC_CMR_ACPA_CLEAR);
	}

	TC_SetRA(tc_num, channel, (dutyCycle*RC_clk));       //Sets the value in RA
	TC_SetRC(tc_num, channel, RC_clk);					//Sets the value in RC

  TC_Start(tc_num, channel);
}

void setup() {

	//sets up the channel masks which is used to enable to correct registers for the PWM to be output on. As PIO controller A, B, C and D are all being 
	//used they must be declared seperately
	unsigned int chanmask_A = (1 << LEFT_MOTOR_COMP_PIOPIN);
	unsigned int chanmask_B = (1 << LEFT_MOTOR_PIOPIN);
	unsigned int chanmask_C = (1 << RIGHT_MOTOR_PIOPIN) | (1 << RIGHT_MOTOR_COMP_PIOPIN);
	unsigned int chanmask_D = (1 << PUMP_PIOPIN);

	Serial.begin(9600);
	pmc_set_writeprotect(false);

	//Serial.print (chanmask, BIN);

	//enable output on PIO controller A, must bit shift a 0 for register ABSR to enable this PIO A register. All other PIO controllers need a 1
	REG_PIOA_PDR = chanmask_A;
	REG_PIOA_ABSR = (0 << LEFT_MOTOR_COMP_PIOPIN);
	REG_PIOA_MDDR = chanmask_A;

	//enable output on PIO controller B
	REG_PIOB_PDR = chanmask_B;
	REG_PIOB_ABSR = chanmask_B;
	REG_PIOB_MDDR = chanmask_B;

	//enable output on PIO controller C
	REG_PIOC_PDR = chanmask_C;
	REG_PIOC_ABSR = chanmask_C;
	REG_PIOC_MDDR = chanmask_C;
	
	//enable output on PIO controller D
	REG_PIOD_PDR = chanmask_D;
	REG_PIOD_ABSR = chanmask_D;
	REG_PIOD_MDDR = chanmask_D;

	//Tells the SAMX3 to connect the clocks to the given TC ID
	pmc_enable_periph_clk(LEFT_MOTOR_ID);
	pmc_enable_periph_clk(LEFT_MOTOR_COMP_ID);
	pmc_enable_periph_clk(RIGHT_MOTOR_ID);
	pmc_enable_periph_clk(RIGHT_MOTOR_COMP_ID);
	pmc_enable_periph_clk(PUMP_ID);
  
	//Sets up all 5 PWM signals but doesn't start them
	pwmwave(50, LEFT_MOTOR_CHAN, LEFT_TC, 0);
	pwmwave(50, LEFT_MOTOR_COMP_CHAN, LEFT_TC, 1);
	pwmwave(50, RIGHT_MOTOR_CHAN, RIGHT_TC, 0);
	pwmwave(50, RIGHT_MOTOR_COMP_CHAN, RIGHT_TC, 1);
	pwmwave(0, PUMP_CHAN, RIGHT_TC, 0);

	//Synchronise all channels from the timer counter specified
	TC0_SYNC = 1;
	TC2_SYNC = 1;
  
}

//puts the left track in reverse and the right in forward for a period and then sets them back at their original values
void turn_left() {

	unsigned int current_duty_cycle = LEFT_MOTOR_RA;
	LEFT_MOTOR_RA = 30;
	LEFT_MOTOR_COMP_RA = 30;
	RIGHT_MOTOR_RA = 70;
	RIGHT_MOTOR_COMP_RA = 70;
	
	delay(1000);
	
	LEFT_MOTOR_RA = current_duty_cycle;
	LEFT_MOTOR_COMP_RA = current_duty_cycle;
	RIGHT_MOTOR_RA = current_duty_cycle;
	RIGHT_MOTOR_COMP_RA = current_duty_cycle;
	
}

//puts the left track in forward and the right in reverse for a period and then sets them back at their original values
void turn_right() {

	unsigned int current_duty_cycle = LEFT_MOTOR_RA;
	LEFT_MOTOR_RA = 70;
	LEFT_MOTOR_COMP_RA = 70;
	RIGHT_MOTOR_RA = 30;
	RIGHT_MOTOR_COMP_RA = 30;
	
	delay(1000);
	
	LEFT_MOTOR_RA = current_duty_cycle;
	LEFT_MOTOR_COMP_RA = current_duty_cycle;
	RIGHT_MOTOR_RA = current_duty_cycle;
	RIGHT_MOTOR_COMP_RA = current_duty_cycle;
	
}

void stop_motors() {
	
	LEFT_MOTOR_RA = 50;
	LEFT_MOTOR_COMP_RA = 50;
	RIGHT_MOTOR_RA = 50;
	RIGHT_MOTOR_COMP_RA = 50;
	
}

void stop_pump() {
	
	PUMP_RA = 0;
	
}

//if motorSelect = 0, change the duty cycle of the left motor. If motorSelect = 1 change the duty cycle of the right motor. If motorSelect = 2, change the duty cycle of both motors
void start_motors(unsigned int duty, int motorSelect) {
	
	double duty_cycle = (double)duty*RC_clk/100;
	
	if (motorSelect == 0) || (motorSelect == 2){
		LEFT_MOTOR_RA = duty_cycle;
		LEFT_MOTOR_COMP_RA = duty_cycle;
	}
	
	if (motorSelect == 1) || (motorSelect == 2){
		RIGHT_MOTOR_RA = duty_cycle;
		RIGHT_MOTOR_COMP_RA = duty_cycle;
	}
	
}

void start_pump(unsigned int duty) {
	
	double duty_cycle = (double)duty*RC_clk/100;
	PUMP_RA = duty_cycle;
	
}

//void dreg(char *s, unsigned int r) {
//	Serial.print(s);
//	Serial.print(r, HEX);
//
//}
//
//void tcregs()
//{
//	dreg("\n CV: ", LEFT_TC->TC_CHANNEL[LEFT_MOTOR_CHAN].TC_CV);
//	dreg(" SR: ", LEFT_TC->TC_CHANNEL[LEFT_MOTOR_CHAN].TC_SR);
//	dreg(" CMR: ", LEFT_TC->TC_CHANNEL[LEFT_MOTOR_CHAN].TC_CMR);
//	dreg(" RA: ", LEFT_TC->TC_CHANNEL[LEFT_MOTOR_CHAN].TC_RA);
//	dreg(" RC: ", LEFT_TC->TC_CHANNEL[LEFT_MOTOR_CHAN].TC_RC);
//}

//If motorSelect = 0, controls the feedback for the left motor. If motorSelect = 1, controls the feedback for the right motor
void feedback(float val, int motorSelect) {

	int floatingDutyCycle = 0;
	
	if (motorSelect == 0){
		floatingDutyCycle = leftFloatingDutyCycle;
	} else {
		floatingDutyCycle = rightFloatingDutyCycle;
	}

	int localDuty = floatingDutyCycle;
	double lower = 0;
	double upper = 0;

//The switch statement determines the upper and lower limits of the feedback required for a given duty cycle
	switch(setDutyCycle) {
		case 70:
			lower = 0.35;
			upper = 0.45;
			break;
		case 75:
			lower = 0.9;
			upper = 1.1;
			break;
		case 80:
			lower = 1.8;
			upper = 2;
			break;
		case 20:
			lower = 1.9;
			upper = 2;
			break;
		case 25:
			lower = 1.15;
			upper = 1.35;
			break;
		case 30:
			lower = 0.7;
			upper = 0.8;
			break;
		default:
			break;
	}

	//Checks to see if the feedback is greater than the upper bound and if the tractor is travelling forward reduce the duty cycle else increase it
	if (val > upper) {
		if (direction == 0){
			localDuty = floatingDutyCycle - 1;
		} else {
			localDuty = floatingDutyCycle + 1;
		}

	//Checks to see if the feedback is less than the lower bound and if the tractor is travelling forward increase the duty cycle else decrease it
	} else if(val < lower) {
		if (direction == 0) {
			localDuty = floatingDutyCycle + 1; 
		} else {
			localDuty = floatingDutyCycle - 1; 
		}
	}

	//If the change in duty cycle is outside the limits of 20%-80% keep it with the bounds
	if (localDuty > 80) {
		localDuty = 80;
	}
	if (localDuty < 20) {
		localDuty = 20;
	}

	//adjust the speed of the motors and update the global variable floatingDutyCycle
	start_motors(localDuty, motorSelect);
	
	if (motorSelect == 0){
		leftFloatingDutyCycle = localDuty;
	} else {
		rightFloatingDutyCycle = localDuty;
	}

}

void loop() {
//	tcregs();
	
	int leftFeedbackVal = 0;
	int rightFeedbackVal = 0;
	
	//if tractor is travelling forward read feedback from forward pins else read from reverse pins
	if (direction == 0){
		leftFeedbackVal = analogRead(leftForwardSensorPin);
		rightFeedbackVal = analogRead(rightForwardSensorPin);
	} else {
		leftFeedbackVal = analogRead(leftReverseSensorPin);
		rightFeedbackVal = analogRead(rightReverseSensorPin);
	}
	
	//Calculate equivlent value from raw feedback values and pass new values to the feedback method
	float  leftOutputVal = (float)leftFeedbackVal*(10.0/1023.0);
	float  rightOutputVal = (float)rightFeedbackVal*(10.0/1023.0);
	
	feedback(leftOutputVal, 0);
	feedback(leftOutputVal, 1);
	
	//Prints the left and right readings and duty cycles
	Serial.print("Left reading: ");
	Serial.print(leftOutputVal);
	Serial.print(", Right reading: ");
	Serial.println(rightOutputVal);
	
	Serial.print("Left duty cycle: ");
	Serial.print(leftFloatingDutyCycle);
	Serial.print("Right duty cycle: ");
	Serial.println(rightFloatingDutyCycle);
	
	delay(300);
	if(Serial.available()){
		setDutyCycle = Serial.parseInt();
		floatingDutyCycle = setDutyCycle;
		start_motors(setDutyCycle);
		start_pump(setDutyCycle);
		//records if tractor is going forwards or backwards
		if (setDutyCycle < 50){
			direction = 1;
		} else {
			direction = 0;
		}
    
	}
	
}

