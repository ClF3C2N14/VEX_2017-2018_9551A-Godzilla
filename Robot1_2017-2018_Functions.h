task Slew() {
while (1) {
	slewCount++;
	for (int i = 0;i<10;i++) {
		int target = motorSetpoint[i]; // set some common values to local vars
		int current = motor[i];
		int new = 0;
		int diff = target-current;
		if (current != target) { // if power is not already on target...
			if (diff < 10 && diff > -10) { // if adjustment is less than 10 power... (prevents adjustment loop)
				new = target; // set new power to target
			}else {
				new = current + (10) * (abs(diff)/(diff)); // adjust new power by 10 in the direction of the target
			}
			motor[i] = new;
		}
	}
	wait1Msec(slewDelay);
}
};

void presetMotorSetpoints() {
	for (int i=0;i<10;i++) {
		motorSetpoint[i] = 0;
	}
}

void setMotor(int mtr, int pwr) { // !REQUIRES NUMERICAL PORT NUMBER!
	pwr = pwr > 0 ? pwr > 127 ? 127 : pwr : pwr < -127 ? -127 : pwr;
	motorSetpoint[mtr-1] = pwr; // set motor port setpoint
};

int liftStabilize(int side) {	//side == 1, left.  side == 0, right.
		//exponential power//
		float ex = btn7l ? 1 : 1.5;
		expoLift = btn6u ? expoLift < 0 ? 0 : expoLift+1 : btn6d ? expoLift > 0 ? 0 : expoLift-1 : 0;
		int liftPower = pow(expoLift,ex);
		int maxUp = btn7l ? 80 : 120;
		int maxDown = btn7l ? -25 : -50;
		liftPower = liftPower > maxUp ? maxUp : liftPower < maxDown ? maxDown : liftPower;
		/******PID*******/
		int power = 0;
			if(side == 1) { // right side master control
				//define constants
				float kp = 25.0;
				float kd = 17.0;
				float ki = 0.005;
				// up makes smaller numbers, so math is backwards
				int error = posLeft-posRight+liftErrorAdjust;
				temp_one = error;
				int leftSpeed = lastPosLeft - posLeft;
				int rightSpeed = lastPosRight - posRight;
				//calculate speed difference (calling it the derivative)
				int derivative = rightSpeed-leftSpeed;
				temp_two = derivative;
				//increment integral
				liftStabIntegral += error;
				//stop integral once error is negated or direction is changed
				if(liftStabIntegral > 0 && error <= 0)
					liftStabIntegral = 0;
				else if(liftStabIntegral < 0 && error >= 0)
					liftStabIntegral = 0;
				//calculate power to the left side
				power = (error*kp) + (derivative*kd) + (liftStabIntegral*ki);
				//limit power values
				int maxDifference = 40;
				if(power > liftPower + maxDifference)
					power = liftPower + maxDifference;
				else if(power < liftPower - maxDifference)
					power  = liftPower - maxDifference;
				//make sure they're going the same direction
				if(power > 0 && liftPower < 0)
					power = liftPower + maxDifference;
				else if(power < 0 && liftPower > 0)
					power = liftPower - maxDifference;
				temp_three = power;
			}else
				power = liftPower;
		/****************/
		//bypassed PID by only using exponential power
return liftPower;
};
//Lift Task Moved to Included Auton File For Merging

/************************
****Select Autonomous****
************************/

const short leftButton = 1;
const short centerButton = 2;
const short rightButton = 4;
int SelectedAuton;
int SelectedAutonSide = 1; // 0 == left, 1 == right

void waitForPress() {
	while(nLCDButtons == 0){}
	sleep(5);
};

void waitForRelease() {
	while(nLCDButtons != 0){}
	sleep(5);
};

int AutonSideSelect(const string A) {
	clearLCDLine(0);
	clearLCDLine(1);
	//display auton that was selected
	displayLCDCenteredString(0, A);
	//display left and right options
	displayLCDString(1,0, "<    Enter    >");
	if(A == "Standard")
		displayLCDString(1,0, "Red  -----  Blue");
	else
		displayLCDString(1,0, "Left  ---  Right");
	waitForPress();
	return nLCDButtons;
	waitForRelease();
}

void AutonSelect() {
	int count = 0;
	bool selected = false;
	int side; //
	clearLCDLine(0);
	clearLCDLine(1);
	//loop while center button is not pressed
	while(!selected) {//nLCDButtons != centerButton) {
		//switch case that allows the user to choose from 4 different options
		switch(count) {
		case 0:
			//Display first choice
			displayLCDCenteredString(0, "10-pt Auton");
			displayLCDString(1,0, "<    Enter    >");
			waitForPress();
			//increment or decrement "count" based on button press
			switch(nLCDButtons) {
			case leftButton:
				waitForRelease();
				count = 3;
				break;
			case rightButton:
				waitForRelease();
				count++;
				break;
			case centerButton:
				waitForRelease();
				//side selection
				side = AutonSideSelect("10-pt Auton");
				if(side != centerButton)
					selected = true;
				break;
			}
			break;
		case 1:
			//DisplaySecondChoice
			displayLCDCenteredString(0, "20-pt Auton");
			displayLCDString(1,0, "<    Enter    >");
			waitForPress();
			//increment or decrement count based on button press
			switch(nLCDButtons) {
			case leftButton:
				waitForRelease();
				count--;
				break;
			case rightButton:
				waitForRelease();
				count++;
				break;
			case centerButton:
				waitForRelease();
				//side selection
				side = AutonSideSelect("20-pt Auton");
				if(side != centerButton)
					selected = true;
				break;
			}
			break;
		case 2:
			//Display first choice
			displayLCDCenteredString(0, "Standard");
			displayLCDString(1,0, "<    Enter    >");
			waitForPress();
			//increment or decrement "count" based on button press
			switch(nLCDButtons) {
			case leftButton:
				waitForRelease();
				count--;
				break;
			case rightButton:
				waitForRelease();
				count++;
				break;
			case centerButton:
				waitForRelease();
				//side selection
				side = AutonSideSelect("Standard");
				if(side != centerButton)
					selected = true;
				break;
			}
			break;
		case 3:
			//Display first choice
			displayLCDCenteredString(0, "Skilllzzzz");
			displayLCDString(1,0, "<    Enter    >");
			waitForPress();
			//increment or decrement "count" based on button press
			switch(nLCDButtons) {
			case leftButton:
				waitForRelease();
				count--;
				break;
			case rightButton:
				waitForRelease();
				count = 0;
				break;
			case centerButton:
				waitForRelease();
				//side selection (removed for skills, only works on the right side)
				side = rightButton;
				selected = true;
				break;
			}
			break;
		default:
			count = 0;
			break;
		}
		SelectedAuton = count+1;
		SelectedAutonSide = side == rightButton ? 1 : side == leftButton ? 0 : 1; // 0 == left, 1 == right
		if(SelectedAuton == 1)
			displayLCDCenteredString(0, "Autonomous 1");//Autonomous 1
		else if(SelectedAuton == 2)
			displayLCDCenteredString(0, "Autonomous 2");
		else if(SelectedAuton == 3)
			displayLCDCenteredString(0, "Autonomous 3");
		else if(SelectedAuton == 4)
			displayLCDCenteredString(0, "Autonomous 4");
		else
			displayLCDCenteredString(0, "***NO AUTON ***");
	}

};

/*****************/
//MAIN FUNCTIONS//
/*****************/
task LCDSelect() {
	while(1) {
		if(nLCDButtons != 0) {
			waitForRelease();
			LCDControl = 1;
			clearLCDLine(0);
			clearLCDLine(1);
			AutonSelect();
			displayLCDCenteredString(1,"Selected");
			sleep(3000);
			LCDControl = 0;
		}
		sleep(100);
	}
}
task LCD() {
	while(1) {
		while(LCDControl)
			sleep(500);
		clearLCDLine(0);                                            // Clear line 1 (0) of the LCD
		clearLCDLine(1);                                            // Clear line 2 (1) of the LCD

		//Display the Primary Robot battery voltage
		displayLCDString(0, 0, "Primary: ");
		sprintf(mainBattery, "%1.2f%c", nImmediateBatteryLevel/1000.0,'V'); //Build the value to be displayed
		displayNextLCDString(mainBattery);

		//Display the expander battery voltage
		displayLCDString(1, 0, "Expander: ");
		sprintf(expanderBattery, "%1.2f%c", SensorValue[PowerExpander]*3.537/1000.0, 'V');    //Build the value to be displayed
		displayNextLCDString(expanderBattery);

		wait1Msec(3000);
		while(LCDControl)
			sleep(500);
		clearLCDLine(0);
		clearLCDLine(1);

		//Display the expander battery voltage
		displayLCDString(0, 0, "Expander: ");
		sprintf(expanderBattery, "%1.2f%c", SensorValue[PowerExpander]*3.537/1000.0, 'V');    //Build the value to be displayed
		displayNextLCDString(expanderBattery);

		//Display the Backup battery voltage
		displayLCDString(1, 0, "Backup: ");
		sprintf(backupBattery, "%1.2f%c", BackupBatteryLevel/1000.0, 'V');	//Build the value to be displayed
		displayNextLCDString(backupBattery);
		sleep(3000);
	}
}

void Buttons() {
	btn5d = vexRT[Btn5D];
	btn5u = vexRT[Btn5U];
	btn6d = vexRT[Btn6D];
	btn6u = vexRT[Btn6U];
	btn8d = vexRT[Btn8D];
	btn8u = vexRT[Btn8U];
	btn8l = vexRT[Btn8L];
	btn8r = vexRT[Btn8R];
	btn7d = vexRT[Btn7D];
	btn7u = vexRT[Btn7U];
	btn7l = vexRT[Btn7L];
	btn7r = vexRT[Btn7R];
};

//user input drive control
void Drive() {
		int rightDrive = vexRT[Ch2] > 15 || vexRT[Ch2] < -15 ? vexRT[Ch2] : 0;
		int leftDrive =  vexRT[Ch3] > 15 || vexRT[Ch3] < -15 ? vexRT[Ch3] : 0;
		//Set Powers
		setMotor(RDrive_A+1,rightDrive);
		setMotor(LDrive_D+1,leftDrive);
		quadRight = SensorValue[RightDriveQuad];
		quadLeft = SensorValue[LeftDriveQuad];
};

bool inRange(int val, int low, int high) {
	bool T;
	if(val >= low && val <= high)
		T = true;
	else
		T = false;
return T;
};

void varWeightedAverage(int var) {
	average = average*weight;
	average = average+var;
	weight++;
	average /= weight;
}

void initGLVar() { // [0] is position, 1,2,or 3 (up,holding,down); [1] is button toggle, [2] derivative Last Error, [3] last time, [4] integral, [5] derivitive
	for(int p=0;p<5;p++) {
		GLStoreVar[p] = 0;
	}
	GLStoreVar[0]++; // increment to match pos comment
	clearTimer(timer1);
}

void GoalLift() {
		goalManual = btn7d || btn8d ? true : btn7u || btn8u ? false : goalManual;
		if(btn7d && goalManual) {
			setMotor(MobileGoalLift+1,127);
		}else if(btn8d && goalManual) {
			setMotor(MobileGoalLift+1,-127);
		}else if(goalManual) {
			setMotor(MobileGoalLift+1,0);
		};
		GLStoreVar[0] = btn8u ? GLStoreVar[1] ? GLStoreVar[0] : 1 : btn7u ? GLStoreVar[1] ? GLStoreVar[0] : 2 : GLStoreVar[0];
		GLStoreVar[1] = btn7u || btn8u ? 1 : 0; // var just like toggleHold
		GLPot = SensorValue[MobileGoalPot]/10;
		int exact = SensorValue[MobileGoalPot];
		int up = 42;
		int down = 227;
		int MGPow = 0;
		int target;
		int error = GLStoreVar[5];
		int deriv;
		float kp = 2.5;
		float kd = 0.9;
		float ki = 0.02;
		if(time1[T1] > 100000) {
			clearTimer(T1);
			GLStoreVar[3] = 0;
		}
		/*********************************/
		if(time1[T1] > GLStoreVar[3]+10 && !goalManual){
			GLStoreVar[3] = time1[T1];
			target = GLStoreVar[0] == 1 ? up : down;
			error = target - GLPot;
			deriv = target*10 - exact - GLStoreVar[2];
			GLStoreVar[2] = target*10-exact;
			GLStoreVar[5] = deriv;
			if(error == 0 || abs(GLStoreVar[4]+error) <= abs(GLStoreVar[4]))
				GLStoreVar[4] = 0;
			else
				GLStoreVar[4] += error;
			if(abs(deriv*kd) > abs(GLStoreVar[4]))
				deriv = -1*GLStoreVar[4]/kd;
			MGPow = (error*kp)+(GLStoreVar[4]*ki)+(deriv*kd);
			MGPow = MGPow > 120 ? 120 : MGPow < -120 ? -120 : MGPow;
			if(GLPot >= down && GLStoreVar[0] == 2) {
				MGPow = 0;
				GLStoreVar[4] = 0;
			}
			if(GLStoreVar[1])
				GLStoreVar[4] = 0;
			if(abs(GLStoreVar[4]) > 25000)
				MGPow = 0;
			if(GLStoreVar[0] == 1 && GLPot <= up)
				MGPow = holding ? -10 : 0;
			setMotor(MobileGoalLift+1, MGPow);
		}
};

void holdLift() {
	int up = btn6u;
	int down = btn6d;
	if(btn6u)
		hold = 1;
	holding = up ? 0 : down ? 0 : hold ? 1 : 0;
	if(btn7r) {
		hold = toggleHold ? hold : hold ? 0 : 1;
		toggleHold = 1;
	}else {
		toggleHold = 0;
	}
};

void claw(){
	//do sensor stuff too
	int close = btn8l ? 1 : 0;
	int open = btn8r ? 1 : 0;
	setMotor(Grabber+1, close && open ? 0 : close ? 127 : open ? -127 : hold ? -7 : 0);
};

void flip() {
	int up = btn5u ? 1 : 0;
	int down = btn5d ? 1 : 0;
	//exponential power curve//
	float ex = .8;
	expoFlip = up ? expoFlip < 0 ? 0 : expoFlip+1 : down ? expoFlip > 0 ? 0 : expoFlip-1 : 0;
	int FlipPower = pow(expoFlip,ex);
	FlipPower = FlipPower > 90 ? 90 : FlipPower < -70 ? -70 : FlipPower;
	if(FlipPower == 0 && holding)
		FlipPower = 10;
	/************************/
	setMotor(Flipper+1,FlipPower);
	setMotor(FlipperRight+1,FlipPower);
}
