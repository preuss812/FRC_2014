
#include "WPILib.h"
#include <cmath>

#define KP 0.006
#define KI 0.0005
#define KD 0.05

#define windingWinchSpeed -0.60
#define unWindingWinchSpeed  0.15

class PIDController812
{
	float errorSum;
	float errorLast;
	
public:
	float Update(float, float);
	void Reset(void);
};

float PIDController812::Update( float goal, float current) {

	float error;
	float errorDelta;
	float PID, p_out, i_out, d_out;
	
	error = goal - current;
	errorSum = errorSum + error;
	errorDelta = error - errorLast;
	errorLast = error;
	
	p_out = KP * error;
	i_out = KI * errorSum;
	d_out = KD * errorDelta;
		
	PID = p_out + i_out + d_out;
		
	return (PID);
};

void PIDController812::Reset() {
	errorSum = 0.0;
	errorLast = 0.0;
};

double PwmLimit(double pwm)
{
	return pwm >= 1.0 ? 1.0 : pwm <= -1.0 ? -1.0 : pwm;
};

double Linearize( double x ) 
{
	const double linearCoeffA =  4.5505;
	const double linearCoeffB = -5.9762;
	const double linearCoeffC =  2.5895;
	const double linearCoeffD = -0.0869;
	const double linearCoeffE =  0.0913;
	
	if (fabs(x) < 0.01) {
		x = 0.0;
	}
	if( x > 0.0 ) {
		return 	linearCoeffA*pow(x,4) + linearCoeffB*pow(x,3) + 
				linearCoeffC*pow(x,2) + linearCoeffD*x +
				linearCoeffE;
	} else if( x < 0.0) {
		return -Linearize(-x);
	} else {
		return 0.0;
	}
};

/**
 * This is a demo program showing the use of the RobotBase class.
 * The SimpleRobot class is the base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls.
 */ 
class RobotDemo : public SimpleRobot
{
//	RobotDrive myRobot; // robot drive system
	DriverStation *m_ds;	// driver station object
	Joystick leftStick; // left
	Joystick rightStick; // right
	Talon *Motor1;	// left front
	Talon *Motor2;	// left rear
	Talon *Motor3; // right rear
	Talon *Motor4; // right front
	Jaguar *Motor5; // winch motor
	RobotDrive *myRobot;
	Relay *brushMotorRelay1;
	Relay *brushMotorRelay2;
	Compressor *compressor;
	DoubleSolenoid *shifter;
	DoubleSolenoid *lifter;
	Counter *leftWheelCounter;
	Counter *rightWheelCounter;
	PIDController812 *leftPID;
	PIDController812 *rightPID;
	
	
public:
	RobotDemo(void):
		leftStick(1),		// as they are declared above.
		rightStick(2) // right stick
	{
//		myRobot.SetExpiration(0.1);  
		Motor1 = new Talon(1); // left rear
		Motor2 = new Talon(2); // left front
		Motor3 = new Talon(3); // right rear
		Motor4 = new Talon(4); // right front
		Motor5 = new Jaguar(5); // winch motor
		brushMotorRelay1 = new Relay(2); // left side
		brushMotorRelay2 = new Relay(3); // right side
		compressor = new Compressor(1,1); // Digital I/O 1, relay 1
		shifter = new DoubleSolenoid(1,2);
		lifter = new DoubleSolenoid(3,4);
		leftWheelCounter = new Counter(3); // Digital I/O 3
		rightWheelCounter = new Counter(2); // Digital I/O 2
		leftPID = new PIDController812;
		rightPID = new PIDController812;

		myRobot = new RobotDrive(Motor1, Motor2, Motor3, Motor4);

/*		myRobot->SetInvertedMotor(RobotDrive::kFrontLeftMotor, true);
		myRobot->SetInvertedMotor(RobotDrive::kRearLeftMotor, true);
		myRobot->SetInvertedMotor(RobotDrive::kFrontRightMotor, true);
		myRobot->SetInvertedMotor(RobotDrive::kRearRightMotor, true);
*/

	}

	/**
	 * Drive left & right motors for 2 seconds then stop
	 */
	void Autonomous(void)
	{
		float max_speed = 0.45;
		float min_speed = 0.10;
		float up_delta_speed = 0.06;
		float down_delta_speed = 0.04;
		float delay = 0.7;
		float duration = 0.0;
		float goal = 110; /* 120 counts == 120 inches == 10 feet */
		float leftPIDOutput, rightPIDOutput, turnRatio;
		float currLeftDist, currRightDist;
		int pidLoopCounter = 0;
		float power;
/*
 * 0.1 Catapult is primed, winch is fully wound
 * 0.2 Ball is in the catapult
 * 0.3 Arms are up to keep the ball in place
 * 1. Start the compressor
 * 1.1 Start the left and right wheel counters, reset them to zero
 * 1.2 Enable a left and right wheel PID controller
 * 2. Drive straight forward and stop at the right place
 * 2.1 Put the chopsticks down
 * 3. Shoot the ball
 * 3.1 Winch down the catapult
 * 3.2 Raise the arms
 * 3.2.1 reset the PID controllers to zero
 * 3.2.2 reset the wheel counters to zero
 * 3.3 Drive back to another ball
 * 3.4 Turn on the chopsticks so that they take the ball in
 * 3.5 Drop the chopsticks to acquire the ball
 * 3.6 Raise the chopsticks to keep the ball
 * 3.7 Stop the chopsticks
 * 3.8 reset wheel counters to zero
 * 3.9 reset the PID controllers to zero error
 * 4.0 drive straight forward and stop at the right place
 * 4.1 drop the arms
 * 4.3 release the winch
*/
		//compressor->Start();
		leftWheelCounter->Start();
		leftWheelCounter->Reset();
		rightWheelCounter->Start();
		rightWheelCounter->Reset();

		myRobot->SetSafetyEnabled(false);
		fprintf(stderr,"Left: %f, Right: %f, goal: %f, lpo: %f, rpo: %f, tr: %f\n", 
				currLeftDist, currRightDist, goal, leftPIDOutput, rightPIDOutput, turnRatio);

		myRobot->Drive(-0.3,-0.06);
		Wait(0.5);
		myRobot->Drive(-0.4,-0.06);
		Wait(0.5);
		
		currLeftDist = leftWheelCounter->Get();
		currRightDist = rightWheelCounter->Get();
		fprintf(stderr,"Left: %f, Right: %f, goal: %f, lpo: %f, rpo: %f, tr: %f\n", 
				currLeftDist, currRightDist, goal, leftPIDOutput, rightPIDOutput, turnRatio);
		
		while(currLeftDist < goal && pidLoopCounter < 100) {
			leftPIDOutput  = leftPID->Update(goal, currLeftDist);
			rightPIDOutput = rightPID->Update(goal, currRightDist);

			if( currLeftDist > currRightDist ) {
				turnRatio = -(currLeftDist - currRightDist)/currLeftDist;
			} else if( currRightDist > currLeftDist ) {
				turnRatio = (currRightDist - currLeftDist)/currRightDist;
			} else {
				turnRatio = 0.0;
			}
			
			if( leftPIDOutput > 0.5) {
				power = 0.5;
			} else {
				power = leftPIDOutput;
			}
			
			myRobot->Drive(PwmLimit(-power), PwmLimit(turnRatio));
			fprintf(stderr,"Left: %f, Right: %f, goal: %f, lpo: %f, rpo: %f, power: %f, tr: %f\n", 
					currLeftDist, currRightDist, goal, leftPIDOutput, rightPIDOutput, power, turnRatio);
			Wait(0.05);
			currLeftDist = leftWheelCounter->Get();
			currRightDist = rightWheelCounter->Get();
			pidLoopCounter++;
		} 
		myRobot->Drive(-0.3, -0.06);
		Wait(0.05);
		myRobot->Drive(-0.2, -0.06);
		Wait(0.025);
/*		
		
		
		for( float v = min_speed; v <= max_speed; v=v+up_delta_speed) {
			fprintf(stderr,"%f: ramp up %f\n",duration, v);
			myRobot->Drive(-v, -0.2);
			Wait(delay);
			duration = duration + delay;
		}
		
		for( float v = max_speed; v >= min_speed; v=v-down_delta_speed) {
			fprintf(stderr,"%f: ramp down %f\n",duration, v);
			myRobot->Drive(v, 0.0);
			Wait(delay);
			duration = duration + delay;
		}
		myRobot->Drive(0.0,0.0);
		Wait(0.5);
		shifter->Set(DoubleSolenoid::kReverse);
		Wait (2.0);
		shifter->Set(DoubleSolenoid::kForward);
		Motor5->SetSpeed(windingWinchSpeed);
		Wait (2.0);
		
		Motor5->SetSpeed(0.0);
*/		
		myRobot->Drive(0.0, 0.0); 	// stop robot
	}
	/**
	 * Runs the motors with arcade steering. 
	 */
	void OperatorControl(void)
	{
		bool armsUp = true;
		double getyPower;
		double getxPower;
		
		DriverStationEnhancedIO &controllerBox =
							DriverStation::GetInstance()->GetEnhancedIO();
		compressor->Start();
		leftWheelCounter->Start();
		leftWheelCounter->Reset();
		rightWheelCounter->Start();
		rightWheelCounter->Reset();
		while (IsOperatorControl())
		{
			fprintf(stderr, "leftWheelCounter=%d, rightWheelCounter=%d\n", 
					leftWheelCounter->Get(), 
					rightWheelCounter->Get());
			
			if(leftWheelCounter->GetStopped()) {
				fprintf(stderr, "leftWheelCounter is stopped\n");
			} else {
				fprintf(stderr, "leftWheelCounter is moving\n");
			}
			if(rightStick.GetRawButton(5)) {
				leftWheelCounter->Reset();
				rightWheelCounter->Reset();
			}
			if(controllerBox.GetDigital(5)) {
				controllerBox.SetDigitalOutput(10,1);
				compressor->Stop();
			} else {
				controllerBox.SetDigitalOutput(10,0);
				compressor->Start();
			}
/*
 			if(controllerBox.GetDigital(4)) {
				myRobot->TankDrive(leftStick,rightStick,true);
				//myRobot->TankDrive(PwmLimit(Linearize( leftStick.GetY())),
					//			   PwmLimit(Linearize(rightStick.GetY())) ); 
				myRobot->TankDrive(leftStick.GetY(), rightStick.GetY());
				controllerBox.SetDigitalOutput(10,1);
			} else {
			*/
				myRobot->ArcadeDrive(rightStick,false);
				/*
				getyPower = PwmLimit(Linearize(rightStick.GetY()));
				getxPower = PwmLimit(Linearize(rightStick.GetX()));
				
			*/
				getyPower = rightStick.GetY();
				getxPower = rightStick.GetX();
				/*
				fprintf(stderr,"x: %f, x': %f, y: %f, y': %f\n",
						rightStick.GetX(), getxPower,
						rightStick.GetY(), getyPower);
*/
				myRobot->ArcadeDrive(getyPower, getxPower);

				/*
				controllerBox.SetDigitalOutput(10,0);
			}
			*/
			if(rightStick.GetRawButton(3) && armsUp == false) {
				shifter->Set(DoubleSolenoid::kForward);
			}
			if(rightStick.GetRawButton(1) && armsUp == false) {
				shifter->Set(DoubleSolenoid::kReverse);
				
			}
			if(leftStick.GetRawButton(1)) {
				//fprintf(stderr,"left stick raw button 1 found\n");
				lifter->Set(DoubleSolenoid::kForward);
				armsUp = false;
				controllerBox.SetDigitalOutput(9,1);
			}
			if(leftStick.GetRawButton(3)) {
				//fprintf(stderr,"left stick raw button 3 found\n");
				lifter->Set(DoubleSolenoid::kReverse);
				armsUp = true; 
				controllerBox.SetDigitalOutput(9,0);
			}
			if(controllerBox.GetDigital(7)) {
				controllerBox.SetDigitalOutput(11,1);
			} else {
				controllerBox.SetDigitalOutput(11,0);
			}
			if(controllerBox.GetDigital(3) && armsUp == false) {
				//fprintf(stderr,"GetDigital(3) is true\n");
			//	controllerBox.SetDigitalOutput(9,1);
				if(controllerBox.GetDigital(7)) {
					Motor5->SetSpeed(unWindingWinchSpeed);
				} else {
					Motor5->SetSpeed(windingWinchSpeed);
				}
			} else {
			//	controllerBox.SetDigitalOutput(9,0);
				Motor5->SetSpeed(0.0);
		
			}
			// left 1&&2 are true
			if(controllerBox.GetDigital(1)&&controllerBox.GetDigital(2)) {
			//	fprintf(stderr,"GetDigital(1) is true: brush reverse\n");
				brushMotorRelay1->Set(Relay::kForward);
				brushMotorRelay2->Set(Relay::kReverse);
			}
			// center 1 is true, 2 is false
			if(controllerBox.GetDigital(1) && !controllerBox.GetDigital(2)) {
			//	fprintf(stderr,"GetDigital(1) && ! GetDigital(2) is true: brush off\n");
				brushMotorRelay1->Set(Relay::kOff);
				brushMotorRelay2->Set(Relay::kOff);
			}
			// right 1 is false, 2 is false
			if(!(controllerBox.GetDigital(1) || controllerBox.GetDigital(2))){
			//	fprintf(stderr, "GetDigital(1) || GetDigital(2) both false: brush forward\n ");
				brushMotorRelay1->Set(Relay::kReverse);
				brushMotorRelay2->Set(Relay::kForward);
			}
			Wait(0.005);				// wait for a motor update time
		}
	}
	
	/**
	 * Runs during test mode
	 */
	void Test() {

	}
};

START_ROBOT_CLASS(RobotDemo);


