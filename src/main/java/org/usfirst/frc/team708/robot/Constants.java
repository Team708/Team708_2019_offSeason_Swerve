package org.usfirst.frc.team708.robot;

//import edu.wpi.first.wpilibj.DoubleSolenoid;
//import edu.wpi.first.wpilibj.Relay;

/**
 * Class containing all the code-related constants, so wiring and
 * gamepad controls are not included
 * @author omn0mn0m
 */
public final class Constants {

	/*
	 * General
	 */

	/*
	 * Drivetrain
	 */
	public static final boolean STEER_MOTOR_INVERTED 					= false;
	public static final double  SWERVE_ROTATION_MAX_SPEED				= 1250.0*8.0; //The 0.8 is to request a speed that is always achievable

	//Swerve Module Steer Offsets (Rotation encoder values when the wheels are facing 0 degrees)
	public static final int STEER_FR_ENCODER_START_POS 					= 835; 	//TODO set correct values
	public static final int STEER_FL_ENCODER_START_POS 					= 2503; //TODO set correct values
	public static final int STEER_BL_ENCODER_START_POS 					= 1371; //TODO set correct values
	public static final int STEER_BR_ENCODER_START_POS 					= 1577; //TODO set correct values

	public static final double	DRIVE_MOTOR_MAX_POWER 					= 1.0;  // 1.0
	public static final double	ROTATE_MOTOR_MAX_POWER 					= 0.5;  // .80
	public static final double	TANK_STICK_TOLERANCE 					= .20;
	public static final double  DRIVETRAIN_WHEEL_DIAMETER 				= 4.0;  //4 inch wheel
	public static final double  DRIVETRAIN_GEAR_RATIO					= 5.0;  //(30/18)*(45/15)

	public static final int STEER_ENCODER_COUNTS_PER_REV          	    = 4096; //set value when known

	public static final double  WHEEL_BASE_LENGTH                       = 17.5; //set value when known
	public static final double  WHEEL_BASE_WIDTH                        = 17.5; //set value when known

	public static final double DRIVE_MOTOR_MAX_ACCEL 					= 572.0; //RPM/S, 16 m/s^2 on 4in wheels

	// PID Tuning parameters
	public static final double driveKp = 0.1;		// Proportional gain
	public static final double driveKi = 0.0;		// Integral gain
	public static final double driveKd = 0.0;		// Derivative gain
	public static final double driveF = 0;		// Feed forward gain
	public static final double driveRampRate = 0.0;	// Ramp rate

	public static final double steerKp = 3.5;		// Proportional gain
	public static final double steerKi = 0.0;		// Integral gain
	public static final double steerKd = 180.0;		// Derivative gain

	public static final int ALLIANCE_RED 	 		= 1;
	public static final int ALLIANCE_BLUE 	 		= -1;

	/*
	 * Driver Assist
	 */
	public static final double ASSIST_DISTANCE		= 16.0;
	public static final double ASSIST_MOVE_SPEED	= .5;
	public static final double ASSIST_TIME			= 1.0;
	
	/*
	 * Smart Dashboard
	 */
	public static final double SEND_STATS_INTERVAL	= .2;	// Interval for reporting in seconds
	public static final boolean DEBUG 				= true;
	   

}
