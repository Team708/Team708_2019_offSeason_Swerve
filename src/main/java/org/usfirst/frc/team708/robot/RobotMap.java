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
	public static final int DtDriveFrontRightMotor  		    = 11; 
	public static final int DtDriveFrontLeftMotor  		    	= 12; 
	
	public static final int DtDriveBackLeftMotor  	            = 13;
	public static final int DtDriveBackRightMotor  	            = 14;

	public static final int DtSteerFrontRightMotor	 	        = 15;
	public static final int DtSteerFrontLeftMotor	 	        = 16;

	public static final int DtSteerBackLeftMotor	 	        = 17;
	public static final int DtSteerBackRightMotor	 	        = 18;



// Elevator 
// 	public static final int elevatorMotorMaster	 			= 21;

// // Ball Intake
// 	public static final int ballIntakeMotorMaster			= 31; 
// //	public static final int ballIntakeMotorSlave1			= 32; 
	
// // Hatch Intake
// //	public static final int hatchIntakeMotorMaster			= 41; 

// // Climber
// 	public static final int ClimberLeftFrontMotor			= 51; 
// 	public static final int ClimberRightFrontMotor			= 52; 

// 	public static final int ClimberLeftRearMotor			= 53; 
// //	public static final int ClimberRightRearMotor			= 54; 
// 	public static final int ClimberRollerMotor				= 55; 


// Digital IO
	public static final int drivetrainEncoderALeft		= 0;  	
	public static final int drivetrainEncoderBLeft		= 1;
	public static final int drivetrainEncoderARight		= 2;
	public static final int drivetrainEncoderBRight		= 3;
//	public static final int 							= 4;
	// public static final int ballSensor					= 5;	
	// public static final int hatchSensor					= 6;
	// // public static final int								= 7;	
	// // public static final int 					 			= 8;  
	public static final int lineSensor					= 9;
	
	
// RELAY
//	public static final int 			 	= 0;
//	public static final int 			 	= 1;
//	public static final int 			 	= 2;
//	public static final int 			 	= 3;
	
// Analog sensor IDs
	// public static final int ballSensor			= 0;
	// public static final int ballSensor			= 1;
//	public static final int 					= 2;
//	public static final int 					= 3;
	
// PCM Ports
	public static final int driveShift		= 0; 	//Shifts Drivetrain to Low gear
//	public static final int 				= 1; 	
//     public static final int intake			= 2;	//intake in and out
//	public static final int					= 3;	
// 	public static final int	hatch			= 4;	//hatch piston in and out
//	public static final int 				= 5;
// 	public static final int beak			= 6;
//	public static final int 		        = 7;
	
	
}
