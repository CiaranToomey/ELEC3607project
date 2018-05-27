/*
 Name:		Motor_code.ino
 Created:	18-May-18 6:37:53 PM
 Author:	Ian
*/

#define		TC0_SYNC					(*(WoReg*) 0x400800C0)		//the address of the sync register used to force TC0 to synchronise all it's channels
#define		TC2_SYNC					(*(WoReg*) 0x400880C0)		//the address of the sync register used to force TC2 to synchronise all it's channels

#define		TIMER_COUNTER1 				41999026		//clock 1 = MCK/2
#define		FREQ			 			100000		//100KHz PWM signal desired for the motors
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

unsigned int RC_clk = TIMER_COUNTER1 / FREQ;			//This is the value that RC needs to be to create the frequency

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

	//TC_Start(LEFT_TC, LEFT_MOTOR_CHAN);
	//TC_Start(LEFT_TC, LEFT_MOTOR_COMP_CHAN);
	//TC_Start(RIGHT_TC, LEFT_MOTOR_CHAN);
	//TC_Start(RIGHT_TC, LEFT_MOTOR_COMP_CHAN);
	//TC_Start(RIGHT_TC, PUMP_CHAN);

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

void start_motors(unsigned int duty) {
	
	double duty_cycle = (double)duty*RC_clk/100;
	LEFT_MOTOR_RA = duty_cycle;
	LEFT_MOTOR_COMP_RA = duty_cycle;
	RIGHT_MOTOR_RA = duty_cycle;
	RIGHT_MOTOR_COMP_RA = duty_cycle;
	
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

void loop() {
//	tcregs();
	
	if(Serial.available()){
		
		start_motors(Serial.parseInt());
		
	}
}

