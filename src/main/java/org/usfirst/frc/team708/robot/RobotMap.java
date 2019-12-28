package org.usfirst.frc.team708.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 *  */
public class RobotMap {
	
// Gamepad USB ports

	public static final int driverGamepad   = 0;
	public static final int operatorGamepad = 1;
	public static final int climbingGamepad = 2;
	
// PWM Ports

//	public static final int 			 	= 0;
//	public static final int 			 	= 1;
//	public static final int  			 	= 2;
//	public static final int  				= 3;
//	public static final int  				= 4;
//	public static final int  				= 5;
//	public static final int  				= 6;
//	public static final int  				= 7;
//	public static final int  				= 8;
//	public static final int  				= 9;
	
// Motor Controler Device IDS

// Drivetrain
	public static final int DRIVE_RIGHT_FRONT_SPARK = 11;
	public static final int DRIVE_LEFT_FRONT_SPARK = 12;
	public static final int DRIVE_LEFT_REAR_SPARK = 13;
	public static final int DRIVE_RIGHT_REAR_SPARK = 14;

	public static final int RIGHT_FRONT_TALON = 15;
	public static final int LEFT_FRONT_TALON = 16;
	public static final int LEFT_REAR_TALON = 17;
	public static final int RIGHT_REAR_TALON = 18;

// Elevator 
	//public static final int elevatorMotorMaster	 			= 21;

// Ball Intake
	//public static final int ballIntakeMotorMaster			= 31; 
//	public static final int ballIntakeMotorSlave1			= 32; 
	
// Hatch Intake
//	public static final int hatchIntakeMotorMaster			= 41; 

// Climber
	//public static final int ClimberLeftFrontMotor			= 51; 
	//public static final int ClimberRightFrontMotor			= 52; 

	//public static final int ClimberLeftRearMotor			= 53; 
//	public static final int ClimberRightRearMotor			= 54; 
	//public static final int ClimberRollerMotor				= 55; 


// Digital IO
	public static final int drivetrainEncoderALeft		= 0;  	
	public static final int drivetrainEncoderBLeft		= 1;
	public static final int drivetrainEncoderARight		= 2;
	public static final int drivetrainEncoderBRight		= 3;
//	public static final int 							= 4;
	// public static final int ballSensor					= 5;	
	//public static final int hatchSensor					= 6;
	// public static final int								= 7;	
	// public static final int 					 			= 8;  
	public static final int lineSensor					= 9;
	
	
// RELAY
//	public static final int 			 	= 0;
//	public static final int 			 	= 1;
//	public static final int 			 	= 2;
//	public static final int 			 	= 3;
	
// Analog sensor IDs
	// public static final int ballSensor			= 0;
	//public static final int ballSensor			= 1;
//	public static final int 					= 2;
//	public static final int 					= 3;
	
// PCM Ports
	public static final int driveShift		= 0; 	//Shifts Drivetrain to Low gear
//	public static final int 				= 1; 	
    //public static final int intake			= 2;	//intake in and out
//	public static final int					= 3;	
	//public static final int	hatch			= 4;	//hatch piston in and out
//	public static final int 				= 5;
	//public static final int beak			= 6;
//	public static final int 		        = 7;
 

  	public static final int LEFT_JOYSTICK = 0;
  	public static final int RIGHT_JOYSTICK = 1;
  	public static final int CONTROLLER = 2;	


	
}
	
