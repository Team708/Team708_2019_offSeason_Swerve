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
	public static final double	DRIVE_MOTOR_MAX_SPEED 					= 1.0;  // 1.0
	public static final double	ROTATE_MOTOR_MAX_SPEED 					= 1.0;  // .80
	public static final double	TANK_STICK_TOLERANCE 					= .20;
	public static final double  DRIVETRAIN_WHEEL_DIAMETER 				= 4.0;  //4inch wheel * (LOW 10.44,  HIGH 4.87)
	public static final double  DRIVETRAIN_GEAR_RATIO					= 5.0;  //(30/18)*(45/15)
	public static final double  DRIVETRAIN_ENCODER_PULSES_PER_REV	 	= 42.0;
	public static final boolean DRIVE_USE_SQUARED_INPUT 				= false;
	public static final boolean DRIVETRAIN_USE_LEFT_ENCODER				= true; // variable to determine which side encoder is on
	public static final double	DRIVE_MOTOR_OFF		 					= 0.0;
	public static final double	GRAYHILL_ENCODER_PULSES_PER_REVOLUTION 	= 256.0;
	public static final double	ENCODER_BOTTOM_POSITION 				= 0.0;

	public static final double STEER_ENCODER_COUNTS_PER_REV             = 360.0; //set value when known

	public static final double  WHEEL_BASE_LENGTH                       = 17.5; //set value when known
	public static final double  WHEEL_BASE_WIDTH                        = 17.5; //set value when known

	public static final double DRIVE_MOTOR_MAX_ACCEL 					= 572.0; //RPM/S, 16 m/s^2 on 4in wheels

	// PID Tuning parameters
	public static final double Kp = 2.0;		// Proportional gain
	public static final double Ki = 0.0;		// Integral gain
	public static final double Kd = 20.0;		// Derivative gain

	public static final double KpForward = 0.1;
	public static final double KiForward = 0.02;
	public static final double KdForward = 0.005;

	public static final double KpBackward = 0.1;
	public static final double KiBackward = 0.02;
	public static final double KdBackward = 0.005;
	
	public static final double pid_tolerance = 1;
	
	/*
	 * Elevator
	 */
	// public static final double 	ELEVATOR_MOTOR_UP		 	=  .8;
	// public static final double 	ELEVATOR_MOTOR_DOWN		 	=  -.3;
	// public static final double 	ELEVATOR_STOP 				= 0.0;
	// public static final double	ELEV_DEADZONE 				=  .5; 
	// 	//Measured off encoder revs
	// public static final double	ELEV_MAX					= 61;
	// // public static final double	ELEV_LVL3				= 58.0; 
	// // public static final double	ELEV_LVL2				= 34.0; 
	// // public static final double	ELEV_LVL1				= 8.0;
	// public static final double	ELEV_CARGO_BALL				= 20.0; 
	// public static final double	ELEV_HATCH_LVL3				= 59.0; 
	// public static final double	ELEV_HATCH_LVL2				= 33.0; 
	// public static final double	ELEV_HATCH_LVL1				= 8.0; 
	// public static final double	ELEV_BALL_LVL1				= 9.0; 
	// public static final double	ELEV_BALL_LVL2				= 35.0; 
	// public static final double	ELEV_BALL_LVL3				= 59.0; 
	// public static final double	ELEV_LVL0					= 7.0; 
	// // public static final double	ELEV_FEEDER					= 8.0; 
	// public static final double	ELEV_GROUND					= 0.0; 
	// public static final int ELEV_ENC_STARTING_POSITION 		= 0;
	// public static final int ELEV_ENC_MIN				 	= 0;
	// public static final int	ELEV_TOLERANCE					= 1;
	// public static final int	ELEV_HOLD_TIMEOUT				= 3;

	// public static final double ELEV_P						= 4.0;  //.8
	// public static final double ELEV_I						= 0;    // .0001
	// public static final double ELEV_D						= 0;    // 1.0
	// public static final double ELEV_Iz						= 0;
	// public static final double ELEV_FF						= 0;

	/*
	 * Intake
	 */
	// public static final double	HATCH_IN			= 1.0;
	// public static final double	HATCH_OUT			= -1.0;
	// public static final double 	HATCH_STOP 			= 0.0;

	// public static final double BALL_IN 				= -.8;
	// public static final double BALL_OUT 			= 0.6;
	// public static final double BALL_STOP 			= 0.0;

	// public static final int	INTAKE_HATCH_IN		=  1;
	// public static final int	INTAKE_HATCH_OUT	=  0;
	
	// public static final int	INTAKE_BALL_IN		=  1;  //0
	// public static final int	INTAKE_BALL_OUT		=  0;  //1

	/*
	 * Cimber
	 */
// 	public static final int		REAR_CLIMBER_ROLLER_FIRST_DISTANCE	=  13;  //inches encoder value
// 	public static final int		REAR_CLIMBER_ROLLER_SECOND_DISTANCE	=  14; //inches encoder value
// //  public static final int		REAR_CLIMBER_ROLLER_DISTANCE_INCHES	=  14; //inches encoder value
// 	public static final int		REAR_CLIMBER_ROLLER_DISTANCE_FINAL	=  7; //inches encoder value
// 	public static final double 	CLIMBER_ROLLER_FORWARD				=  0.4;		
// 	public static final double 	CLIMBER_ROLLER_BACKWARD				=  -0.2;		

// 	public static final int		MOVE_CLIMBER_FORWARD				=	18;
// 	public static final double	MOVE_CLIMBER_TOLERANCE				=  	.5;
// 	public static final double	MOVE_CLIMBER_FRONT_EXTEND			=  	-1.0;
// 	public static final double	MOVE_CLIMBER_FRONT_RETRACT			=  	1.0;
// 	public static final double	MOVE_CLIMBER_REAR_EXTEND			=  	-0.69;
// 	public static final double	MOVE_CLIMBER_REAR_RETRACT			=	1.0;
// 	public static final double 	HAB_LVL3_ENCODER_DISTANCE		=	42500;
// 	public static final double	HAB_LVL2_ENCODER_DISTANCE		=	15000;
// 	public static final double	STOP_CLIMBER						= 	0.0;

// 	public static final double ROLLER_GEAR_RATIO					= 16.0;	//16:1
// 	public static final double ROLLER_DIAMETER						= 2.5;	//Diameter in inches
// 	public static final int ROLLER_ENCODER_COUNTS_PER_REV			= 12;	//12 ticks, lol

	/*
	 * Vision
	 */
	// public static final int VISION_LED_ON				= 0;
	// public static final int VISION_LED_OFF				= 1;
	// public static final int VISION_LED_BLINK			= 2;
	// public static final int VISION_PIPELINE_0			= 0;
	// public static final int VISION_PIPELINE_1			= 1;
	// public static final int VISION_TARGET_NOT_FOUND		= 0;
	// public static final int VISION_TARGET_FOUND			= 1;

	// public static final double TARGET_Y 				=   0.0;		
	// public static final double Y_THRESHOLD				=   1.0;	//Target angle in degrees
	// public static final double X_THRESHOLD 				=    .6;    //Target angle in degrees

	// public static final double AREA_THRESHOLD				= 	2.0;
	// public static final double ROCKET_HATCH_TARGET_AREA		=	10.0;
	// public static final double ROCKET_CARGO_TARGET_AREA		=	10.0;


	// public static final double VISION_MOVE			= .4;
	// public static final double VISION_ROTATE		= .2;

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
