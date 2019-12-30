package org.usfirst.frc.team708.robot;

import edu.wpi.first.wpilibj.buttons.*;
import org.usfirst.frc.team708.robot.util.Gamepad;
import org.usfirst.frc.team708.robot.util.triggers.*;
// import org.usfirst.frc.team708.robot.commands.climber.*;
// import org.usfirst.frc.team708.robot.commands.climber.SetClimbLvl2;
// import org.usfirst.frc.team708.robot.commands.drivetrain.*;
// import org.usfirst.frc.team708.robot.commands.elevator.*;
// import org.usfirst.frc.team708.robot.commands.intake.*;
// import org.usfirst.frc.team708.robot.commands.driverAssist.*;
// import org.usfirst.frc.team708.robot.commands.visionProcessor.*;


/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */

public class OI {
	
// Gamepads
	public final static Gamepad driverGamepad 	= new Gamepad(RobotMap.driverGamepad);	// Driver gamepad
	public final static Gamepad operatorGamepad = new Gamepad(RobotMap.operatorGamepad);// Operator gamepad
	public final static Gamepad climbingGamepad = new Gamepad(RobotMap.climbingGamepad);// Operator gamepad
	
// look in Gamepad.java for button constants
	
/*
 * Driver Button Assignment
 */

// Driver Buttons
 	private static final int FIELD_ORIENTED_DRIVE_BUTTON    = Gamepad.button_R_Shoulder;
// 	private static final int SHIFT_DRIVETRAIN_HIGH_BUTTON	= Gamepad.button_L_Shoulder;
//	private static final int HOLDGEARHIGH					= Gamepad.shoulderAxisLeft;;
//	private static final int 								= Gamepad.shoulderAxisRight;;
// 	private static final int ROLLER_BACKWARDS 				= Gamepad.button_Y;
// 	private static final int FIND_BALL_BUTTON				= Gamepad.button_B;
// 	private static final int ROLLER_FORWARDS				= Gamepad.button_A;
	
	/*
	* Operator Button Assignment
	*/
	
	// private static final int FIND_TAPE_BUTTON			= Gamepad.button_X;
	
//	private static final int 								= Gamepad.shoulderAxisLeft;;
//	private static final int 								= Gamepad.shoulderAxisRight;

// private static final int HATCH_OUT_BUTTON				= Gamepad.button_L_Shoulder;
// private static final int HATCH_IN_BUTTON				= Gamepad.button_R_Shoulder;

// private static final int BALL_OUT_BUTTON				= Gamepad.button_R_Shoulder;
// private static final int BALL_IN_BUTTON					= Gamepad.button_L_Shoulder;


//	private static final int GROUND_BUTTON					= Gamepad.button_X;
	// private static final int LEVEL_0_BUTTON					= Gamepad.button_X;
	// private static final int LEVEL_1_ROCKET_BUTTON			= Gamepad.button_A;
	// private static final int LEVEL_2_ROCKET_BUTTON			= Gamepad.button_B;
	// private static final int LEVEL_3_ROCKET_BUTTON			= Gamepad.button_Y;

	// private static final int ELEVATOR_OVERIDE_BUTTON		= Gamepad.leftStick_Y;

	// private static final int TOGGLE_INTAKE_BUTTON			= Gamepad.button_Back;
	// private static final int TOGGLE_HATCH_BUTTON			= Gamepad.button_Start;
	// private static final int TOOGLE_BEAK_BUTTON				= Gamepad.button_RightStick;
	// private static final int CARGOSHIP_BUTTON				= Gamepad.button_LeftStick;

/**
 * Climber Buttons
 */
	// private static final int INITIATE_CLIMB					= Gamepad.button_Start;	
	// private static final int STOP_CLIMB						= Gamepad.button_Back;	
	// private static final int CLIMBER_FRONT_BUTTON			= Gamepad.leftStick_Y;
	// private static final int CLIMBER_REAR_BUTTON			= Gamepad.rightStick_Y;
	// private static final int CLIMBER_FORWARD_BUTTON			= Gamepad.button_R_Shoulder;	
	// private static final int CLIMBER_BACKWARD_BUTTON		= Gamepad.button_L_Shoulder;	
	// private static final int SET_HAB_LVL2					= Gamepad.button_A;
	// private static final int SET_HAB_LVL3					= Gamepad.button_B;

//	private static final int OPENBUTTON1_BUTTON				= Gamepad.button_Back;
// 	private static final int OPENBUTTON2_BUTTON				= Gamepad.button_Start;
//	private static final int CLIMBER_BUTTON					= Gamepad.leftStick_Y;
//	private static final int CLIMBER_FORWARD_BUTTON			= Gamepad.rightStick_X;	

/*
 * Driver Button events
 */
	 public static final Button fieldOrientLock		= new JoystickButton(driverGamepad, FIELD_ORIENTED_DRIVE_BUTTON);
	// public static final Button highGear		 	= new JoystickButton(driverGamepad, SHIFT_DRIVETRAIN_HIGH_BUTTON);
	// public static final Button findBall			= new JoystickButton(driverGamepad, FIND_BALL_BUTTON);
	// public static final Button rollerForward	= new JoystickButton(driverGamepad, ROLLER_FORWARDS);
	// public static final Button rollerBackward	= new JoystickButton(driverGamepad, ROLLER_BACKWARDS);
	// public static final Button findTape			= new JoystickButton(driverGamepad, FIND_TAPE_BUTTON);
	//	public static final Trigger highGearEngaged	= new AxisUp(driverGamepad, HOLDGEARHIGH);
	//	public static final Trigger highGearRelease	= new AxisDown(driverGamepad, HOLDGEARHIGH);
	//	public static final Trigger lowGear			= new AxisUp(driverGamepad, HOLDGEARHIGH);
	//	public static final Trigger lowGear			= new AxisDown(driverGamepad, HOLDGEARHIGH);
	
	/*
	* Operator Button events
	*/
	// public static final Button hatchIn			= new JoystickButton(operatorGamepad, HATCH_IN_BUTTON);
	// public static final Button hatchOut			= new JoystickButton(operatorGamepad, HATCH_OUT_BUTTON);
	// public static final Button ballIn			= new JoystickButton(operatorGamepad, BALL_IN_BUTTON);
	// public static final Button ballOut			= new JoystickButton(operatorGamepad, BALL_OUT_BUTTON);
	
	// public static final Button level0			= new JoystickButton(operatorGamepad, LEVEL_0_BUTTON);
	// public static final Button cargoship		= new JoystickButton(operatorGamepad, CARGOSHIP_BUTTON);
	// public static final Button level1Rocket		= new JoystickButton(operatorGamepad, LEVEL_1_ROCKET_BUTTON);
	// public static final Button level2Rocket		= new JoystickButton(operatorGamepad, LEVEL_2_ROCKET_BUTTON);
	// public static final Button level3Rocket		= new JoystickButton(operatorGamepad, LEVEL_3_ROCKET_BUTTON);
	
	// public static final Button toggleIntake		= new JoystickButton(operatorGamepad, TOGGLE_INTAKE_BUTTON);
	// public static final Button toggleHatch		= new JoystickButton(operatorGamepad, TOGGLE_HATCH_BUTTON);
	// public static final Button toggleBeak		= new JoystickButton(operatorGamepad, TOOGLE_BEAK_BUTTON);
	// //	public static final Button level0			= new JoystickButton(operatorGamepad, LEVEL_0_ELEV_BUTTON);

	// public static final Trigger elevatorUp		= new AxisUp(operatorGamepad, ELEVATOR_OVERIDE_BUTTON);
	// public static final Trigger elevatorDown	= new AxisDown(operatorGamepad, ELEVATOR_OVERIDE_BUTTON);


	// public static final Button initiateClimb	= new JoystickButton(climbingGamepad, INITIATE_CLIMB);
	// public static final Button stopClimb		= new JoystickButton(climbingGamepad, STOP_CLIMB);
	// // public static final Trigger climbUp			= new AxisUp(climbingGamepad, 	CLIMBER_BUTTON);
	// // public static final Trigger climbDown		= new AxisDown(climbingGamepad, CLIMBER_BUTTON);
	// public static final Trigger climbUpFront	= new AxisUp(climbingGamepad, CLIMBER_FRONT_BUTTON);
	// public static final Trigger climbDownFront	= new AxisDown(climbingGamepad, CLIMBER_FRONT_BUTTON);
	// public static final Trigger climbUpRear		= new AxisUp(climbingGamepad, CLIMBER_REAR_BUTTON);
	// public static final Trigger climbDownRear	= new AxisDown(climbingGamepad, CLIMBER_REAR_BUTTON);
	// public static final Button climbForward		= new JoystickButton(climbingGamepad, CLIMBER_FORWARD_BUTTON);
	// public static final Button climbBackward	= new JoystickButton(climbingGamepad, CLIMBER_BACKWARD_BUTTON);
	// public static final Button setHABLvl2 		= new JoystickButton(climbingGamepad, SET_HAB_LVL2);
	// public static final Button setHABLvl3 		= new JoystickButton(climbingGamepad, SET_HAB_LVL3);

	public OI() {

// Driver
		// highGear.whileHeld(new GearHigh());
		// highGear.whenReleased(new GearLow());
		// findBall.whenPressed(new FindBall());
		// findFeeder.whenPressed(new FindFeederCG());				
		// rollerForward.whileHeld(new MoveRollerForward());	
		// rollerBackward.whileHeld(new MoveRollerBackward());	
		// findTape.whenPressed(new FindRocket());
		
//Operator
		// hatchIn.whileHeld(new IntakeHatchIn());
		// hatchOut.whileHeld(new IntakeHatchOut());
		// ballIn.whileHeld(new IntakeBallIn());
		// ballOut.whileHeld(new IntakeBallOut());
		// toggleHatch.whenPressed(new ToggleHatch());
		// toggleIntake.whenPressed(new ToggleIntake());
		// toggleBeak.whenPressed(new ToggleBeak());

	//	eleGround.whenPressed(new ElevatorToGround());
		// cargoship.whenPressed(new CargoshipCG());
		// cargoship.whenPressed(new ElevatorToCargoship());
		// level0.whenPressed(new Level0CG());
	//  level0.whenPressed(new ElevatorToLevel0());

		// level1Rocket.whenPressed(new ElevatorToLevel1());
		// level2Rocket.whenPressed(new ElevatorToLevel2());
		// level3Rocket.whenPressed(new ElevatorToLevel3());
		// level1Rocket.whenPressed(new Level1CG());				
		// level2Rocket.whenPressed(new Level2CG());				
		// level3Rocket.whenPressed(new Level3CG());				
						
// 		initiateClimb.whenPressed(new InitiateClimbCG());
// 		stopClimb.whenPressed(new ClimberStop());

// 		elevatorUp.whileActive(new JoystickMoveElevator());
// 		elevatorDown.whileActive(new JoystickMoveElevator());

// //Climber
// 		setHABLvl2.whenPressed(new SetClimbLvl2());
// 		setHABLvl3.whenPressed(new SetClimbLvl3());
// 		climbUpFront.whileActive(new JoystickMoveClimberFront());
// 		climbDownFront.whileActive(new JoystickMoveClimberFront());
// 		climbUpRear.whileActive(new JoystickMoveClimberRear());
// 		climbDownRear.whileActive(new JoystickMoveClimberRear());
// 		climbForward.whileHeld(new MoveRollerForward());	
// 		climbBackward.whileHeld(new MoveRollerBackward());	

		// climbUp.whileActive(new JoystickMoveClimb());
		// climbDown.whileActive(new JoystickMoveClimb());
		
		// climbForward.whileActive(new JoystickMoveClimb()); 
		// climbBack.whileActive(new JoystickMoveClimb()); 

/*
 		.whileActive(new 
		.whenPressed(new
		.whileHeld(new
		.whenReleased(new 
*/
		}
}

