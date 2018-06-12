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
int heading = 0;							//0 means forward, 1 means reverse
int feedbackSelect = 0;       //0 means open loop, 1 means closed loop
int printMessage = 0;          //0 means no message, 1 means low water, 2 means obstacle detected

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
void ISR_func_Water() {
  stop_motors();
  stop_pump();
  printMessage = 1;
  
}
void ISR_func_IRled() {
  stop_motors();
  stop_pump();
  printMessage = 2;
  
}

void print_message(){

  if (printMessage == 1){
    Serial.println("Emergency stop, LOW WATER LEVELS");
  } else if (printMessage == 2){
    Serial.println("Emergency stop, OBSTACLE DETECTED");
  }
  printMessage = 0;
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
  pinMode(26, INPUT);
  pinMode(32,INPUT);
  
  attachInterrupt(26, ISR_func_IRled, RISING);
  attachInterrupt(32, ISR_func_Water, FALLING);
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
	pwmwave(1, PUMP_CHAN, RIGHT_TC, 0);

	//Synchronise all channels from the timer counter specified
	TC0_SYNC = 1;
	TC2_SYNC = 1;
  
}

//puts the left track in reverse and the right in forward for a period and then sets them back at their original values
void turn_left() {

	unsigned int current_duty_cycle = LEFT_MOTOR_RA;
  double forward_duty_cycle = (double)75*RC_clk/100;
  double reverse_duty_cycle = (double)25*RC_clk/100;
	LEFT_MOTOR_RA = reverse_duty_cycle;
	LEFT_MOTOR_COMP_RA = reverse_duty_cycle;
	RIGHT_MOTOR_RA = forward_duty_cycle;
	RIGHT_MOTOR_COMP_RA = forward_duty_cycle;
	
	delay(1000);
	
	LEFT_MOTOR_RA = current_duty_cycle;
	LEFT_MOTOR_COMP_RA = current_duty_cycle;
	RIGHT_MOTOR_RA = current_duty_cycle;
	RIGHT_MOTOR_COMP_RA = current_duty_cycle;
	
}

//puts the left track in forward and the right in reverse for a period and then sets them back at their original values
void turn_right() {

	unsigned int current_duty_cycle = LEFT_MOTOR_RA;
  double forward_duty_cycle = (double)75*RC_clk/100;
  double reverse_duty_cycle = (double)25*RC_clk/100;
	LEFT_MOTOR_RA = forward_duty_cycle;
	LEFT_MOTOR_COMP_RA = forward_duty_cycle;
	RIGHT_MOTOR_RA = reverse_duty_cycle;
	RIGHT_MOTOR_COMP_RA = reverse_duty_cycle;
	
	delay(1000);
	
	LEFT_MOTOR_RA = current_duty_cycle;
	LEFT_MOTOR_COMP_RA = current_duty_cycle;
	RIGHT_MOTOR_RA = current_duty_cycle;
	RIGHT_MOTOR_COMP_RA = current_duty_cycle;
	
}

void stop_motors() {

  double duty_cycle = (double)50*RC_clk/100;
	LEFT_MOTOR_RA = duty_cycle;
	LEFT_MOTOR_COMP_RA = duty_cycle;
	RIGHT_MOTOR_RA = duty_cycle;
	RIGHT_MOTOR_COMP_RA = duty_cycle;
  setDutyCycle = 50;
  leftFloatingDutyCycle = 50;
	rightFloatingDutyCycle = 50;
  
}

void stop_pump() {

  double duty_cycle = (double)1*RC_clk/100;
	PUMP_RA = duty_cycle;
	
}

//if motorSelect = 0, change the duty cycle of the left motor. If motorSelect = 1 change the duty cycle of the right motor. If motorSelect = 2, change the duty cycle of both motors
void start_motors(unsigned int duty, int motorSelect) {
	
	double duty_cycle = (double)duty*RC_clk/100;
	
	if ((motorSelect == 0) || (motorSelect == 2)){
		LEFT_MOTOR_RA = duty_cycle;
		LEFT_MOTOR_COMP_RA = duty_cycle;
	}
	
	if ((motorSelect == 1) || (motorSelect == 2)){
		RIGHT_MOTOR_RA = duty_cycle;
		RIGHT_MOTOR_COMP_RA = duty_cycle;
	}
	
}

void start_pump(unsigned int duty) {
	
	double duty_cycle = (double)duty*RC_clk/100;
	PUMP_RA = duty_cycle;
	
}

//If motorSelect = 0, controls the feedback for the left motor. If motorSelect = 1, controls the feedback for the right motor
void feedback(int motorSelect) {
  
  int analogueInput = 0;
  int localDuty = 0;
  
  if ((motorSelect == 0) && (heading == 0)){
    analogueInput = analogRead(leftForwardSensorPin);
    localDuty = leftFloatingDutyCycle;
  } else if  ((motorSelect == 0) && (heading == 1)){
    analogueInput = analogRead(leftReverseSensorPin);
    localDuty = leftFloatingDutyCycle;
  } else if  ((motorSelect == 1) && (heading == 0)){
    analogueInput = analogRead(rightForwardSensorPin);
    localDuty = rightFloatingDutyCycle;
  } else {
    analogueInput = analogRead(rightReverseSensorPin);
    localDuty = rightFloatingDutyCycle;
  }

  //Prints the left and right readings and duty cycles
  if (motorSelect == 0){
    Serial.print("Left reading: ");
    Serial.print(analogueInput);
    Serial.print(", Left duty cycle: ");
    Serial.println(leftFloatingDutyCycle);
  } else {
    Serial.print("Right reading: ");
    Serial.print(analogueInput);
    Serial.print(", Right duty cycle: ");
    Serial.println(rightFloatingDutyCycle);
  }
  
  double lower = 0;
  double upper = 0;

  if ((setDutyCycle > 30) && (setDutyCycle <70)){
    return;
  }
  
//The switch statement determines the upper and lower limits of the feedback required for a given duty cycle
  switch(setDutyCycle) {
    case 70:
      lower = 140;
      upper = 155;
      break;
    case 75:
      lower = 200;
      upper = 240;
      break;
    case 80:
      lower = 270;
      upper = 300;
      break;
    case 30:
      lower = 110;
      upper = 140;
      break;
    case 25:
      lower = 200;
      upper = 220;
      break;
    case 20:
      lower = 240;
      upper = 270;
      break;
    default:
      break;
  }

  //Checks to see if the feedback is greater than the upper bound and if the tractor is travelling forward reduce the duty cycle else increase it
  if (analogueInput > upper) {
    if (heading == 0){
      localDuty = localDuty - 1;
    } else {
      localDuty = localDuty + 1;
    }

  //Checks to see if the feedback is less than the lower bound and if the tractor is travelling forward increase the duty cycle else decrease it
  } else if(analogueInput < lower) {
    if (heading == 0) {
      localDuty = localDuty + 1; 
    } else {
      localDuty = localDuty - 1; 
    }
  }

  //If the change in duty cycle is outside the limits of 20%-85% keep it with the bounds
  if (localDuty > 85) {
    localDuty = 85;
  }
  if (localDuty < 15) {
    localDuty = 15;
  }

  //adjust the speed of the motors and update the global variable floatingDutyCycle
  start_motors(localDuty, motorSelect);
  
  if (motorSelect == 0){
    leftFloatingDutyCycle = localDuty;
  } else {
    rightFloatingDutyCycle = localDuty;
  }
}

void parse_command(){
  int count = 0;
  char firstCommand;
  char secondCommand;
  char number [2];
  int val = 0;
  while(Serial.available() > 0) { 
    if (count == 0) {
      firstCommand = (char)Serial.read();
    } 
    if (count == 1) {
      secondCommand = (char)Serial.read();
    }
    if (count > 1) {
      number[count-2] = Serial.read();
    }
    count++;
  }
  Serial.println();
  Serial.println("received data");
  val = atoi(number);       
  
  if (firstCommand == 'D' && secondCommand == 'C') {
    start_motors(val,2);
    setDutyCycle = val;
    leftFloatingDutyCycle = val;
    rightFloatingDutyCycle = val;
    //records if tractor is going forwards or backwards
    if (val < 50){
      heading = 1;
    } else {
      heading = 0;
    }
  }
  if (firstCommand == 'S' && secondCommand == 'T') {
    stop_motors();
    stop_pump();
  }
  if (firstCommand == 'T' && secondCommand == 'R') {
    turn_right();
  }
  if (firstCommand == 'T' && secondCommand == 'L') {
    turn_left();
  }
  if (firstCommand == 'S' && secondCommand == 'P') {
    start_pump(val);
  }
  if (firstCommand == 'F' && secondCommand == 'B') {
    feedbackSelect = val;
  }
}

void loop() {
  print_message();
  if (feedbackSelect == 1){
    //Check feedback for left motor
  	feedback(0);
    //check feedback for right motor
  	feedback(1);
  } else {
    Serial.print("Duty cycle: ");
    Serial.println(setDutyCycle);
  }
	
   delay(300);
   if (Serial.available()){
      parse_command();
    }
}

