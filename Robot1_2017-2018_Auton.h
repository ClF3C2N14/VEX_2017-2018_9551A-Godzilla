int liftTarget;
int lastLiftError;
int liftError;
int liftIntegral;
int liftPower;
int liftPID = 0;
bool liftOnTarget = false;
int driveTarget[2]; // 0 == left, 1 == right
bool driveOnTarget = false;
bool ChainBarUp = false;
bool ChainBarOnTarget = true;
int drivePID = 0;

/*DrivePID Vars*/
	int d_error[2]; // 0 == left, 1 == right
	int d_lastError[2];
	int d_power[2];
	int d_integral[2];
	int d_derivative = 0;
/***************/

void setLift(int power) {
	setMotor(LiftOne_B+1, power);
	setMotor(LiftTwo_C+1, power);
}

void AutonLiftPID() {
	/*****Calculate Lift Position*****/
		float temp_left;
		float temp_right;
		float power;
		int derivative;
		/************PID Stuff*************/
		float  pid_Kp = .8;
		float pid_Kd = 10;
		float pid_Ki = 0.005;
		int maxError = 80;
		int minError = 20;
		temp_left = SensorValue[LiftPotLeft];
		temp_right = SensorValue[LiftPotRight] + rightAdjust;
			posLeft = temp_left;
			posRight = temp_right;
			lastLiftError = liftError;
			int lift = posRight;//0.5*(posLeft+posRight);
			liftError = lift-liftTarget;
			derivative = liftError-lastLiftError;
			liftIntegral += liftError;
			/************************************/
			power = (pid_Kp * liftError) + (pid_Kd * derivative) + (pid_Ki * liftIntegral);
			if(power < -20)
				power *= .5;
			power = power > 110 ? 110 : power < -30 ? -30 : power;
			if(abs(liftError) < minError){
				liftIntegral = 0;
				power = 12;
				liftOnTarget = true;
				if(derivative > 3){
					power = 15;
				}else if(derivative < -3) {
					power = 5;
				}else {
					holding = 1;
				};
			}else if(liftError < 0 && abs(liftError) < minError+150 && derivative > 6) {
				power = 10;
				liftIntegral = 0;
				timer ++;
			};//else if(holding) {
			//	if(abs(liftError) < minError+20)
			//		power = 12;
			//	else
			//		holding = 0;
			//}
			liftPower = power;
		if(liftPID) {
			setLift(power);
		}else {
			liftIntegral = 0;
		}
}

task Lift() {
	//vars
		float temp_left;
		float temp_right;
		int power[2];
	while(1) {
		/*****Calculate Lift Position*****/// and some other stuff...
		temp_left = SensorValue[LiftPotLeft];
		temp_right = SensorValue[LiftPotRight] + rightAdjust;
		lastPosLeft = posLeft;
		lastPosRight = posRight;
		posLeft = temp_left;
		posRight = temp_right;
		lift_one = motorSetpoint[LiftOne_B];
		lift_two = motorSetpoint[LiftTwo_C];
		/************************************/
		if(liftAuton) {
			while(liftPID && liftAuton) {
				AutonLiftPID();
				sleep(15);
			}
		}else if(btn6u || btn6d) {
			for(int i=0;i<2;i++)
				power[i] = liftStabilize(i);
			setMotor(LiftOne_B+1, power[0] > 0 ? power[0]+5 : power[0]-5);//Right
			setMotor(LiftTwo_C+1, power[1]); //Left
		}else if(holding) {
			liftStabIntegral = 0;
			for(int i=0;i<2;i++)
				power[i] = 12; //liftHold(i);
			setMotor(LiftOne_B+1, power[0]);//Right
			setMotor(LiftTwo_C+1, power[1]);//Left
		}else {
			liftStabIntegral = 0;
			setMotor(LiftOne_B+1, 0);
			setMotor(LiftTwo_C+1, 0);
		}
		sleep(20);
	}
};

void setDrive(int leftDrive, int rightDrive) {
	setMotor(RDrive_A+1,rightDrive);
	setMotor(LDrive_D+1,leftDrive);
}

task AutonDrivePID() {
	float pid_Kp = 0.4;
	float pid_Kd = 4.5;
	float pid_Ki = 0.001;
	int i = 0;
	bool test;
	int maxError = 300;
	int minError = 15;
	bool rightOnTarget = false;
	bool leftOnTarget = false;
	while(1) {
		quadLeft = SensorValue[LeftDriveQuad];
		quadRight = SensorValue[RightDriveQuad];
		for(i=0;i<2;i++) {
			d_error[i] = !i ? driveTarget[0]-quadLeft : driveTarget[1] - quadRight;
			d_integral[i] += d_error[i];
			test = d_integral[i] > 0 && d_error[i] < 0 ? true : d_integral[i] < 0 && d_error[i] > 0 ? true : false;
			if(d_error == 0 || test || abs(d_error[i]) > maxError)
				d_integral[i] = 0;
			d_derivative = d_error[i] - d_lastError[i];
			d_lastError[i] = d_error[i];
			d_power[i] = (d_error[i]*pid_Kp) + (d_derivative*pid_Kd) + (d_integral[i]*pid_Ki);
			if(abs(d_power[i]) < 20 && abs(d_error[i]) > minError)
				d_power[i] = d_power[i] > 0 ? d_power[i]+10 : d_power[i]-10;
			if(abs(d_power[i]) < 10 && abs(d_error[i]) > minError)
				d_power[i] = d_power[i] > 0 ? d_power[i]+15 : d_power[i]-15;
			if(abs(d_error[i]) < minError) {
				d_power[i] = 0;
				d_integral[i] = 0;
				rightOnTarget = i ? true : false;
				leftOnTarget = !i ? true : false;
			}
		}
		driveOnTarget = rightOnTarget && leftOnTarget ? true : false;
		if(drivePID)
			setDrive(d_power[0],d_power[1]);
		else
			setDrive(0,0);
		sleep(15);
	}
}

void liftWait() {
	sleep(500);
	while(!liftOnTarget && liftAuton){
		sleep(100);
	};
}

void initializeQuadEncoders() {
	SensorValue[RightDriveQuad] = 0;
	SensorValue[LeftDriveQuad] = 0;
	driveTarget[0] = 0;
	driveTarget[1] = 0;
}

void setClaw(int power) {
	setMotor(Grabber+1,power);
}

void closeClaw() {
	setClaw(-70);
	sleep(500);
	setClaw(-8);
};

void openClaw(int wait = 300) {
	setClaw(60);
	sleep(wait);
	setClaw(0);
};

void drive(int leftAdjust, int rightAdjust) {
	drivePID = 1;
	if(!SelectedAutonSide) { // 0 == left, 1 == right -- right default (I like to program on the right side)
		//basically, swap the values for the right and left sides.
		driveTarget[0] += rightAdjust; // these are the swapped sides and values
		driveTarget[1] += leftAdjust;
	}else {
		driveTarget[0] += leftAdjust; // these are the correct sides and values
		driveTarget[1] += rightAdjust;
	}
};

void setFlipThing(int power){
	setMotor(Flipper+1,power);
	setMotor(FlipperRight+1,power);
};

void flipThing(int power,int time) {
	setFlipThing(power);
	sleep(time);
	setFlipThing(0);
}

void liftGoal() {
	GLStoreVar[0] = 1;
}

void lowerGoal() {
	GLStoreVar[0] = 2;
}

task ChainBar() {
	while(1) {
		while(ChainBarUp) {
			if(!ChainBarOnTarget){
				flipThing(120,600);
				ChainBarOnTarget = true;
			}
			sleep(50);
		};
		while(!ChainBarUp) {
			if(!ChainBarOnTarget){
				flipThing(-100,300);
				ChainBarOnTarget = true;
			}
			sleep(50);
		};
		sleep(50);
	};
};

task AutonGoalLift() {
	GLStoreVar[0] = 1;
	while(1) {
		GLPot = SensorValue[MobileGoalPot]/10;
		int exact = SensorValue[MobileGoalPot];
		int up = 45;
		int down = 230;
		int MGPow = 0;
		int target;
		int error = GLStoreVar[5];
		int deriv;
		float kp = 2.5;
		float kd = 0.9;
		float ki = 0.02;
		/*********************************/
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
			if(GLStoreVar[0] == 1 && MGPow == 0)
				MGPow = -10;
			setMotor(MobileGoalLift+1, MGPow);
		sleep(60);
	}

}

void liftToPos(int target) {
	liftPID = 1;
	liftOnTarget = false;
	target = target < 50 ? 50 : target;
	target *= 10;
	liftTarget = target;
}

void flipThingUp() {
	ChainBarOnTarget = false;
	ChainBarUp = true;
}

void flipThingDown() {
	ChainBarOnTarget = false;
	ChainBarUp = false;
}

void stopLift(int pos = 200) {
	liftToPos(pos);
	sleep(300);
	liftPID = 0;
	setLift(0);
}

void liftPowerSet(int power) {
	liftPID = 0;
	setLift(power);
}

task stack() {
	startTask(ChainBar);
	sleep(500);
	liftAuton = true;
	int maxHeight = 12; // max number of cones to stack
	int coneHeight = 8;
	int liftHeight;
	int t_one;
	int t_two;
	int loadPos = 160;
	setDrive(-5,-5);
	for(int p = 0;p<maxHeight;p++) {
		t_one = p-1;
		t_two = coneHeight*t_one;
		liftHeight = p<1 ? 205 : 195-t_two;
		//start by going to the position to grab the driver load
		if(p >= 1) {
			closeClaw();
			liftToPos(loadPos);
		}
		if(p < 3) {
			flipThingUp();
			if(p == 0)
				sleep(50);
			sleep(100*(p+5));
			liftToPos(liftHeight);
			sleep(400);
			if(p==0)
				sleep(50);
			openClaw(400);
			liftToPos(155);
			sleep(300);
			flipThingDown();
			liftWait();
			liftToPos(loadPos);
			liftWait();
			sleep(300);
		}else {
			liftPowerSet(127);
			sleep(50+80*(p-7));
			flipThingUp();
			sleep(200);
			liftToPos(liftHeight-20);
			sleep(600);
			liftToPos(liftHeight);
			liftWait();
			if(p+1 == maxHeight)
				break;
			openClaw(400);
			liftToPos(liftHeight-25);
			sleep(200);
			flipThingDown();
			sleep(500);
			liftToPos(loadPos);
			liftWait();
		}
	};
	setDrive(0,0);
	stacking = false;
	liftPID = 0;
	liftAuton = false;
	stopTask(ChainBar);
	stopTask(stack);
}

void abort() {
	buttons();
	if(btn7l || btn8d) {
		stacking = false;
		liftPID = 0;
		liftAuton = false;
		stopTask(stack);
	};
}

void startTime() {
	clearTimer(T1);
	timer = 0;
}

void stopTime() {
	timer = time1[T1];
}

void stopAuton(){
	stacking = false;
	liftPID = 0;
	liftAuton = false;
	drivePID = 0;
	setLift(0);
};

void AutonOne() { // 10-pt Auton
	startTime();
	liftToPos(170);
	lowerGoal();
	closeClaw();
	drive(1900,1900);
	sleep(2000);
	liftGoal();
	flipThing(-80,100);
	setFlipThing(-10);
	sleep(1600);
	stopLift();
	openClaw(300);
	//grab second cone

	liftToPos(170);
	openClaw(200);
	flipThingDown();
	sleep(300);
	liftToPos(200);
	sleep(300);
	drive(300,300);
	sleep(200);
	closeClaw();
	LiftToPos(170);
	flipThingUp();
	sleep(700);
	liftToPos(200);
	sleep(300);
	openClaw(300);

	//third cone
	liftToPos(170);
	openClaw(200);
	flipThingDown();
	sleep(300);
	liftToPos(200);
	sleep(300);
	drive(300,300);
	sleep(200);
	closeClaw();
	liftToPos(170);
	//score mobile goal in 10
	drive(-2600,-2600);
	flipThingUp();
	sleep(800);
	liftPID = 0;
	setLift(0);
	sleep(1600);
	drive(500,-1300);
	sleep(1500);
	drive(800,800);
	openClaw(400);
	liftToPos(170);
	lowerGoal();
	sleep(1500);
	drive(-800,-800);
	sleep(500);
	liftGoal();
	sleep(1500);
	stopLift();
	stopTime();
	stopAuton();
}

void AutonTwo() { // 20-pt Auton
	startTime();
	liftToPos(170);
	lowerGoal();
	closeClaw();
	drive(1900,1900);
	sleep(1900);
	liftGoal();
	flipThing(-80,100);
	setFlipThing(-10);
	sleep(1600);
	stopLift();
	openClaw(300);
	//grab second cone
	liftToPos(170);
	openClaw(200);
	flipThingDown();
	sleep(300);
	liftToPos(200);
	sleep(500);
	drive(300,300);
	sleep(100);
	closeClaw();
	//score mobile goal in 20
	drive(-2300,-2300);
	flipThingUp();
	liftToPos(170);
	sleep(800);
	stopLift();
	sleep(1400);
	drive(300,-300);
	openClaw(300);
	sleep(200);
	drive(-700,-700);
	sleep(1000);
	drive(500,-500);
	sleep(600);
	drive(1200,1200);
	liftToPos(170);
	sleep(800);
	lowerGoal();
	sleep(1500);
	drive(-800,-800);
	liftGoal();
	sleep(1500);
	stopLift();
	stopTime();
	stopAuton();
}

void AutonThree() { // Standard
	closeClaw();
	drive(-480,480);
	sleep(1000);
	liftToPos(110);
	flipThingDown();
	liftWait();
	drive(500,500);
	sleep(1000);
	liftToPos(130);
	sleep(300);
	openClaw();
	flipThingUp();
	stopLift();
	sleep(500);
	stopAuton();
}

void AutonFour() { // Skilllzzz
	//first goal
	liftToPos(160);
	closeClaw();
	lowerGoal();
	sleep(1500);
	drive(1300,1300);
	sleep(1000);
	liftGoal();
	sleep(1500);
	stopLift();
	sleep(500);
	openClaw(600);
	drive(-1000,-1000);
	sleep(1000);
	drive(-800,800);
	sleep(1000);
	drive(700,900);
	sleep(900);
	drive(1000,1000);
	liftToPos(120);
	sleep(800);
	lowerGoal();
	sleep(1500);
	drive(-900,-900);
	sleep(1000);
	//second goal
	drive(650,-650);
	sleep(1000);
	drive(2000,2000);
	sleep(2000);
	liftToPos(130);
	sleep(300);
	liftGoal();
	sleep(1500);
	drive(-2000,-2000);
	sleep(2000);
	drive(-650,650);
	sleep(1000);
	drive(400,400);
	lowerGoal();
	sleep(1500);
	drive(-800,-800);
	liftGoal();
	sleep(1700);
	stopLift();
}

void AutonTest() {
	liftToPos(170);
}
