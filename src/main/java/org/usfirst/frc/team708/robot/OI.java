package org.usfirst.frc.team708.robot;

import edu.wpi.first.wpilibj.buttons.*;

//import com.analog.adis16448.frc.ADIS16448_IMU.Axis;

import org.usfirst.frc.team708.robot.util.Gamepad;
//import org.usfirst.frc.team708.robot.util.triggers.*;


/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */

public class OI {
	
// Gamepads
	public final static Gamepad driverGamepad 	= new Gamepad(RobotMap.driverGamepad);	// Driver gamepad
	public final static Gamepad operatorGamepad = new Gamepad(RobotMap.operatorGamepad);// Operator gamepad
	
// look in Gamepad.java for button constants
	
/*
 * Driver Button Assignment
 */

	public static final int TRANSLATION_X 					= Gamepad.leftStick_X;
	public static final int TRANSLATION_Y					= Gamepad.leftStick_Y;
	public static final int ROTATION						= Gamepad.rightStick_X;
// 	public static final int 								= Gamepad.rightStick_Y;

 	private static final int FIELD_ORIENTED_DRIVE_BUTTON    = Gamepad.button_R_Shoulder;
// 	private static final int 								= Gamepad.button_L_Shoulder;
//	private static final int 								= Gamepad.shoulderAxisLeft;;
//	private static final int 								= Gamepad.shoulderAxisRight;
// 	private static final int 								= Gamepad.button_X;
 	private static final int FIELD_ORIENT_RESET				= Gamepad.button_Y;
// 	private static final int 								= Gamepad.button_B;
// 	private static final int 								= Gamepad.button_A;

/*
* Operator Button Assignment
*/

// 	private static final int 								= Gamepad.leftStick_X;
// 	private static final int 								= Gamepad.leftStick_Y;
// 	private static final int 								= Gamepad.rightStick_X;
// 	private static final int 								= Gamepad.rightStick_Y;

// 	private static final int 								= Gamepad.button_X;
// 	private static final int 								= Gamepad.button_Y;
// 	private static final int 								= Gamepad.button_A;
// 	private static final int 								= Gamepad.button_B;
// 	private static final int 								= Gamepad.button_L_Shoulder;
// 	private static final int 								= Gamepad.button_R_Shoulder;
//	private static final int 								= Gamepad.shoulderAxisLeft;;
//	private static final int 								= Gamepad.shoulderAxisRight;
// 	private static final int 								= Gamepad.button_Back;
// 	private static final int 								= Gamepad.button_Start;
// 	private static final int 								= Gamepad.button_RightStick;
// 	private static final int 								= Gamepad.button_LeftStick;

/*
 * Driver Button events
 */
	public static final Button fieldOrientLock	= new JoystickButton(driverGamepad, FIELD_ORIENTED_DRIVE_BUTTON);
 	public static final Button fieldOrientReset	= new JoystickButton(driverGamepad, FIELD_ORIENT_RESET);
// 	public static final Button 					= new JoystickButton(driverGamepad, ROLLER_BACKWARDS);
// 	public static final Button 					= new JoystickButton(driverGamepad, FIND_TAPE_BUTTON);
//	public static final Trigger 				= new AxisUp(driverGamepad, HOLDGEARHIGH);
//	public static final Trigger 				= new AxisDown(driverGamepad, HOLDGEARHIGH);
//	public static final Trigger 				= new AxisUp(driverGamepad, HOLDGEARHIGH);
//	public static final Trigger 				= new AxisDown(driverGamepad, HOLDGEARHIGH);

/*
* Operator Button events
*/
// public static final Button hatchIn			= new JoystickButton(operatorGamepad, HATCH_IN_BUTTON);
// public static final Button hatchOut			= new JoystickButton(operatorGamepad, HATCH_OUT_BUTTON);
// public static final Button ballIn			= new JoystickButton(operatorGamepad, BALL_IN_BUTTON);
// public static final Button ballOut			= new JoystickButton(operatorGamepad, BALL_OUT_BUTTON);
	
// public static final Button level0			= new JoystickButton(operatorGamepad, LEVEL_0_BUTTON);
// public static final Button cargoship			= new JoystickButton(operatorGamepad, CARGOSHIP_BUTTON);
// public static final Button level1Rocket		= new JoystickButton(operatorGamepad, LEVEL_1_ROCKET_BUTTON);
// public static final Button level2Rocket		= new JoystickButton(operatorGamepad, LEVEL_2_ROCKET_BUTTON);
// public static final Button level3Rocket		= new JoystickButton(operatorGamepad, LEVEL_3_ROCKET_BUTTON);
	
// public static final Button toggleIntake		= new JoystickButton(operatorGamepad, TOGGLE_INTAKE_BUTTON);
// public static final Button toggleHatch		= new JoystickButton(operatorGamepad, TOGGLE_HATCH_BUTTON);
// public static final Button toggleBeak		= new JoystickButton(operatorGamepad, TOOGLE_BEAK_BUTTON);
//	public static final Button level0			= new JoystickButton(operatorGamepad, LEVEL_0_ELEV_BUTTON);

// public static final Trigger elevatorUp		= new AxisUp(operatorGamepad, ELEVATOR_OVERIDE_BUTTON);
// public static final Trigger elevatorDown		= new AxisDown(operatorGamepad, ELEVATOR_OVERIDE_BUTTON);

// public static final Button initiateClimb		= new JoystickButton(climbingGamepad, INITIATE_CLIMB);
// public static final Button stopClimb			= new JoystickButton(climbingGamepad, STOP_CLIMB);
// public static final Trigger climbUp			= new AxisUp(climbingGamepad, 	CLIMBER_BUTTON);
// public static final Trigger climbDown		= new AxisDown(climbingGamepad, CLIMBER_BUTTON);
// public static final Trigger climbUpFront		= new AxisUp(climbingGamepad, CLIMBER_FRONT_BUTTON);
// public static final Trigger climbDownFront	= new AxisDown(climbingGamepad, CLIMBER_FRONT_BUTTON);
// public static final Trigger climbUpRear		= new AxisUp(climbingGamepad, CLIMBER_REAR_BUTTON);
// public static final Trigger climbDownRear	= new AxisDown(climbingGamepad, CLIMBER_REAR_BUTTON);
// public static final Button climbForward		= new JoystickButton(climbingGamepad, CLIMBER_FORWARD_BUTTON);
// public static final Button climbBackward		= new JoystickButton(climbingGamepad, CLIMBER_BACKWARD_BUTTON);
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
						
		//	initiateClimb.whenPressed(new InitiateClimbCG());
		//	stopClimb.whenPressed(new ClimberStop());

		// 	elevatorUp.whileActive(new JoystickMoveElevator());
		// 	elevatorDown.whileActive(new JoystickMoveElevator());

	/*
 		.whileActive(new 
		.whenPressed(new
		.whileHeld(new
		.whenReleased(new 
	*/
	}
}

