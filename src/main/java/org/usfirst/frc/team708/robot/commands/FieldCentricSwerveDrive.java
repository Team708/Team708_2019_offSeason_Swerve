/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team708.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team708.robot.Robot;
import org.usfirst.frc.team708.robot.subsystems.Drivetrain;
import org.usfirst.frc.team708.robot.OI;

public class FieldCentricSwerveDrive extends Command {
	
	public static final double ROTATION_SCALE = 1.0 / 30.0;
	public static final double DEADZONE = 0.05;

	private double originHeading;
  	private double leftPow = 1.0;
	private double rightPow = 1.0;
	
	public FieldCentricSwerveDrive() {
        requires(Robot.drivetrain);
    }
	
	@Override
	protected void initialize() {
		 originHeading = Robot.zeroHeading;
	}

    @Override
	protected void execute() {
		if (OI.fieldOrientReset.get()){
			originHeading = Robot.imu.getAngle();
		}

		double translationX = Math.pow(Math.abs(OI.driverGamepad.getAxis(OI.TRANSLATION_X)), leftPow) * Math.signum(OI.driverGamepad.getAxis(OI.TRANSLATION_X));
		double translationY = Math.pow(Math.abs(OI.driverGamepad.getAxis(OI.TRANSLATION_Y)), leftPow) * -Math.signum(OI.driverGamepad.getAxis(OI.TRANSLATION_Y));
		double rotation = Math.pow(Math.abs(OI.driverGamepad.getAxis(OI.ROTATION)), rightPow) * Math.signum(OI.driverGamepad.getAxis(OI.ROTATION)) * ROTATION_SCALE;
		
		// double translationX = Math.pow(Math.abs(OI.driverGamepad.getAxis(Gamepad.leftStick_X)), leftPow) * Math.signum(OI.driverGamepad.getAxis(Gamepad.leftStick_X));
		// double translationY = Math.pow(Math.abs(OI.driverGamepad.getAxis(Gamepad.leftStick_Y)), leftPow) * -Math.signum(OI.driverGamepad.getAxis(Gamepad.leftStick_Y));
        // double omega = Math.pow(Math.abs(OI.driverGamepad.getAxis(Gamepad.rightStick_X)), rightPow) * Math.signum(OI.driverGamepad.getAxis(Gamepad.rightStick_X)) * OMEGA_SCALE;
       		
        // Add a small deadzone on the joysticks
        if (Math.abs(translationX) < Math.pow(DEADZONE, leftPow)) translationX = 0.0;
		if (Math.abs(translationY) < Math.pow(DEADZONE, leftPow)) translationY = 0.0;
		if (Math.abs(rotation) < Math.pow(DEADZONE, rightPow) * ROTATION_SCALE) rotation = 0.0;
		
		
		// If all of the joysticks are in the deadzone, don't update the motors
		// This makes side-to-side strafing much smoother
		if (translationX == 0.0 && translationY == 0.0 && rotation == 0.0) {
			Robot.drivetrain.setDriveFL(0.0);
			Robot.drivetrain.setDriveBL(0.0);
			Robot.drivetrain.setDriveFR(0.0);
			Robot.drivetrain.setDriveBR(0.0);
			return;
   		}	

		if (OI.fieldOrientLock.get()) { // TODO: add back in not (!). Removed for testing.
        	// When the Left Joystick trigger is not pressed, The robot is in Field Centric Mode.
        	// The calculations correct the translationY and translationX values for field centric attitude. 
    		
    		// Rotate the velocity vector from the joystick by the difference between our
    		// current orientation and the current origin heading
    		double originCorrection = Math.toRadians(originHeading - Robot.imu.getAngle());
    		double temp = translationY * Math.cos(originCorrection) - translationX * Math.sin(originCorrection);
    		translationX = translationX * Math.cos(originCorrection) + translationY * Math.sin(originCorrection);
    		translationY = temp;
		}
		
		
        Drivetrain.set(translationX, translationY, rotation);
    }

    @Override
	protected boolean isFinished() {
        return false;
    }

    @Override
	protected void end() {
    }

    @Override
	protected void interrupted() {
    	end();
    }

}