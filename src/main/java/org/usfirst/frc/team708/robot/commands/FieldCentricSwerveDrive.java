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
import org.usfirst.frc.team708.robot.util.Gamepad;

public class FieldCentricSwerveDrive extends Command {
	
	public static final double OMEGA_SCALE = 1.0 / 30.0;
	public static final double DEADZONE = 0.05;

	private double originHeading = 0.0;
	private double originCorr = 0;
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
		if (OI.driverGamepad.getRawButtonPressed(7)){
			originHeading = Robot.imu.getYaw();
		}

		double originOffset = 360 - originHeading;
		originCorr = Robot.imu.getYaw() + originOffset;

		double strafe = Math.pow(Math.abs(OI.driverGamepad.getAxis(Gamepad.leftStick_X)), leftPow) * Math.signum(OI.driverGamepad.getAxis(Gamepad.leftStick_X));
		double forward = Math.pow(Math.abs(OI.driverGamepad.getAxis(Gamepad.leftStick_Y)), leftPow) * -Math.signum(OI.driverGamepad.getAxis(Gamepad.leftStick_Y));
        double omega = Math.pow(Math.abs(OI.driverGamepad.getAxis(Gamepad.rightStick_X)), rightPow) * Math.signum(OI.driverGamepad.getAxis(Gamepad.rightStick_X)) * OMEGA_SCALE;
       		
        // Add a small deadzone on the joysticks
        if (Math.abs(strafe) < Math.pow(DEADZONE, leftPow)) strafe = 0.0;
		if (Math.abs(forward) < Math.pow(DEADZONE, leftPow)) forward = 0.0;
		if (Math.abs(omega) < Math.pow(DEADZONE, rightPow) * OMEGA_SCALE) omega = 0.0;
		
		
		// If all of the joysticks are in the deadzone, don't update the motors
		// This makes side-to-side strafing much smoother
		if (strafe == 0.0 && forward == 0.0 && omega == 0.0) {
			Robot.drivetrain.setDriveLeftFront(0.0);
			Robot.drivetrain.setDriveLeftRear(0.0);
			Robot.drivetrain.setDriveRightFront(0.0);
			Robot.drivetrain.setDriveRightRear(0.0);
			return;
   		}	

		if (!OI.driverGamepad.getTrigger()) {
        	// When the Left Joystick trigger is not pressed, The robot is in Field Centric Mode.
        	// The calculations correct the forward and strafe values for field centric attitude. 
    		
    		// Rotate the velocity vector from the joystick by the difference between our
    		// current orientation and the current origin heading
    		double originCorrection = Math.toRadians(originHeading - Robot.imu.getYaw());
    		double temp = forward * Math.cos(originCorrection) - strafe * Math.sin(originCorrection);
    		strafe = strafe * Math.cos(originCorrection) + forward * Math.sin(originCorrection);
    		forward = temp;
		}
		
		
        Drivetrain.calculateMeasurements(strafe, forward, omega);
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