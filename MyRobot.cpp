
#include "WPILib.h"
#include <cmath>

#define KP 5/10
#define KI 0
#define KD 0

class PIDController812
{
	float prev_err, integral_err;
public:
	float PIDCalc(float, float);
	void Reset(void);
};

float PIDController812::PIDCalc( float setPoint, float currentPoint) {
	float error;
	float delta_err;
	float p_out;
	float i_out;
	float d_out;
	float output;
		
	error = currentPoint - setPoint;
	delta_err = prev_err - error;
	integral_err += error;
	
	fprintf(stderr, "PID data: error %f, delta_err %f, integral_err %f\n",
			error,
			delta_err,
			integral_err);
	
	p_out = error * KP;
	i_out = integral_err * KI;
	d_out = delta_err * KD;
		
	output = p_out + i_out + d_out;
		
	return (output);
};

void PIDController812::Reset() {
	prev_err = 0.0;
	integral_err = 0.0;
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
		return 0.0;
	} else if( x > 0.0 ) {
		return 	linearCoeffA*pow(x,4) + linearCoeffB*pow(x,3) + 
				linearCoeffC*pow(x,2) + linearCoeffD*x +
				linearCoeffE;
	} else {
		return -Linearize(-x);
	}
	return 0.0; // should never execute this statement
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
	Jaguar *Motor1;	// left front
	Jaguar *Motor2;	// left rear
	Jaguar *Motor3; // right rear
	Jaguar *Motor4; // right front
	Jaguar *Motor5; // winch motor
	RobotDrive *myRobot;
	Relay *brushMotorRelay1;
	Relay *brushMotorRelay2;
	Compressor *compressor;
	DoubleSolenoid *shifter;
	DoubleSolenoid *lifter;
	PIDController812 *pid_controller_2;
	
	
public:
	RobotDemo(void):
		leftStick(1),		// as they are declared above.
		rightStick(2) // right stick
	{
//		myRobot.SetExpiration(0.1);  
		Motor1 = new Jaguar(1); // left rear
		Motor2 = new Jaguar(2); // left front
		Motor3 = new Jaguar(3); // right rear
		Motor4 = new Jaguar(4); // right front
		Motor5 = new Jaguar(5); // winch motor
		brushMotorRelay1 = new Relay(2); // left side
		brushMotorRelay2 = new Relay(3); // right side
		compressor = new Compressor(1,1); // Digital I/O 1, relay 1
		shifter = new DoubleSolenoid(1,2);
		lifter = new DoubleSolenoid(3,4);
		pid_controller_2 = new PIDController812;

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
		float max_speed = 0.5;
		float min_speed = 0.10;
		float up_delta_speed = 0.03;
		float down_delta_speed = 0.04;
		float delay = 0.4;
		float duration = 0.0;
		
		compressor->Start();
		myRobot->SetSafetyEnabled(false);
		for( float v = min_speed; v <= max_speed; v=v+up_delta_speed) {
			fprintf(stderr,"%f: ramp up %f\n",duration, v);
			myRobot->Drive(v, 0.0);
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
		Motor5->SetSpeed(-0.25);
		Wait (2.0);
		Motor5->SetSpeed(0.0);
		
		//	myRobot.Drive(0.0, 0.0); 	// stop robot
	}
	/**
	 * Runs the motors with arcade steering. 
	 */
	void OperatorControl(void)
	{
		DriverStationEnhancedIO &controllerBox =
							DriverStation::GetInstance()->GetEnhancedIO();
		compressor->Start();
		while (IsOperatorControl())
		{
			if(controllerBox.GetDigital(4)) {
		//		myRobot->TankDrive(leftStick,rightStick,true);
				myRobot->TankDrive(PwmLimit(Linearize( leftStick.GetY())),
								   PwmLimit(Linearize(rightStick.GetY())) ); 
				controllerBox.SetDigitalOutput(10,1);
			} else {
				fprintf(stderr,"x: %f, x': %f, y: %f, y': %f\n",
						rightStick.GetX(), PwmLimit(Linearize(rightStick.GetX())),
						rightStick.GetY(), PwmLimit(Linearize(rightStick.GetY())));
				
			//	myRobot->ArcadeDrive(rightStick,false);
				myRobot->ArcadeDrive(PwmLimit(Linearize(rightStick.GetY())),
									 PwmLimit(Linearize(rightStick.GetX())) );
				controllerBox.SetDigitalOutput(10,0);
			}
			if(rightStick.GetRawButton(3)) {
				shifter->Set(DoubleSolenoid::kForward);
			}
			if(rightStick.GetRawButton(1)) {
				shifter->Set(DoubleSolenoid::kReverse);
			}
			if(leftStick.GetRawButton(3)) {
				fprintf(stderr,"left stick raw button 1 found\n");
				lifter->Set(DoubleSolenoid::kForward);
			}
			if(leftStick.GetRawButton(1)) {
				fprintf(stderr,"left stick raw button 3 found\n");
				lifter->Set(DoubleSolenoid::kReverse);
			}
			if(controllerBox.GetDigital(3)) {
				fprintf(stderr,"GetDigital(3) is true\n");
				controllerBox.SetDigitalOutput(9,1);
				Motor5->SetSpeed(-0.25);
			} else {
				controllerBox.SetDigitalOutput(9,0);
				Motor5->SetSpeed(0.0);
			}
			// left 1&&2 are true
			if(controllerBox.GetDigital(1)&&controllerBox.GetDigital(2)) {
				fprintf(stderr,"GetDigital(1) is true: brush reverse\n");
				brushMotorRelay1->Set(Relay::kForward);
				brushMotorRelay2->Set(Relay::kReverse);
			}
			// center 1 is true, 2 is false
			if(controllerBox.GetDigital(1) && !controllerBox.GetDigital(2)) {
				fprintf(stderr,"GetDigital(1) && ! GetDigital(2) is true: brush off\n");
				brushMotorRelay1->Set(Relay::kOff);
				brushMotorRelay2->Set(Relay::kOff);
			}
			// right 1 is false, 2 is false
			if(!(controllerBox.GetDigital(1) || controllerBox.GetDigital(2))){
				fprintf(stderr, "GetDigital(1) || GetDigital(2) both false: brush forward\n ");
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


