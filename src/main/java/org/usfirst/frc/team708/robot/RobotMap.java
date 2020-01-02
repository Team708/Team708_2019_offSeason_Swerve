package org.usfirst.frc.team708.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 *  */
public class RobotMap {
	
/*
 *Gamepad USB ports
 */
	public static final int driverGamepad   = 0;
	public static final int operatorGamepad = 1;
	
/*
 * Motor Controler Device CAN IDs
 */ 
// Drivetrain
	public static final int DtDriveFrontRightMotor  		    = 11; 
	public static final int DtDriveFrontLeftMotor  		    	= 12; 
	
	public static final int DtDriveBackLeftMotor  	            = 13;
	public static final int DtDriveBackRightMotor  	            = 14;

	public static final int DtSteerFrontRightMotor	 	        = 15;
	public static final int DtSteerFrontLeftMotor	 	        = 16;

	public static final int DtSteerBackLeftMotor	 	        = 17;
	public static final int DtSteerBackRightMotor	 	        = 18;

/*
 * PCM Port IDs
 */
//	public static final int					= 0;
//	public static final int 				= 1;
//  public static final int					= 2;
//	public static final int					= 3;
// 	public static final int					= 4;
//	public static final int 				= 5;
// 	public static final int					= 6;
//	public static final int 		        = 7;

/*
 * RoboRIO port IDs
 */ 
// Digital IO
// 	public static final int 				= 0;
// 	public static final int 				= 1;
// 	public static final int					= 2;
// 	public static final int					= 3;
//	public static final int 				= 4;
//	public static final int					= 5;
//	public static final int					= 6;
// 	public static final int					= 7;
// 	public static final int 				= 8;
//	public static final int					= 9;

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
	
// RELAY
//	public static final int 			 	= 0;
//	public static final int 			 	= 1;
//	public static final int 			 	= 2;
//	public static final int 			 	= 3;
	
// Analog sensor IDs
// 	public static final int					= 0;
//	public static final int					= 1;
//	public static final int 				= 2;
//	public static final int 				= 3;

}
