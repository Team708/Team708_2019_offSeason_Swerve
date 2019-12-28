package org.usfirst.frc.team708.robot;

import org.usfirst.frc.team708.robot.util.Gamepad;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;


/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
	
	public class OI{

// Gamepads
	public final static Gamepad driverGamepad 	= new Gamepad(RobotMap.driverGamepad);	// Driver gamepad
	public final static Gamepad operatorGamepad = new Gamepad(RobotMap.operatorGamepad);// Operator gamepad
	public final static Gamepad climbingGamepad = new Gamepad(RobotMap.climbingGamepad);// Operator gamepad

  public Joystick leftJoy;
  public Joystick rightJoy;
  public XboxController controller;

  public  OI(){
    leftJoy = new Joystick(RobotMap.LEFT_JOYSTICK);
    rightJoy = new Joystick(RobotMap.RIGHT_JOYSTICK);
    controller = new XboxController(RobotMap.CONTROLLER);
  }
  public boolean getAButtonPress(){
    return controller.getAButtonPressed();
  }

  public boolean getAButtonRelease(){
      return controller.getAButtonReleased();
  }

  public boolean getBButtonn(){
      return controller.getBButton();
  }

  public boolean getBButtonPress(){
      return controller.getBButtonPressed();
  }

  public boolean getBButtonRelease(){
      return controller.getBButtonReleased();
  }

  public boolean getLeftJoyButton (int buttonNumber){
      return leftJoy.getRawButton(buttonNumber);
  }

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());

	// private static final int TRANSLATION			= Gamepad.button_LeftStick;
	// private static final int ROTATION				= Gamepad.button_RightStick;

}


