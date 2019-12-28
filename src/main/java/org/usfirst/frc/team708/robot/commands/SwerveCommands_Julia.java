// package org.usfirst.frc.team708.robot.commands.drivetrain;

// import org.usfirst.frc.team708.robot.Robot;

// import edu.wpi.first.wpilibj.command.Command;

// public class SwerveCommands_Julia extends Command {

// 	public static final double OMEGA_SCALE = 1.0 / 30.0;
// 	public static final double DEADZONE = 0.05;

// 	private double originHeading = 0.0;
// 	//private double originCorr = 0;
// 	private double leftPow = 1.0;
// 	private double rightPow = 1.0;

// 	public SwerveCommands_Julia() {
//         requires(Robot.drivetrain);
//     }
	
// 	@Override
// 	protected void initialize() {
// 	}

//     @Override
// 	protected void execute() {

// 		double originOffset = 360 - originHeading;

// 		double speed = Math.pow(Math.abs(Robot.oi.leftJoy.getX()), leftPow) * Math.signum(Robot.oi.leftJoy.getX());
// 		double translation = Math.pow(Math.abs(Robot.oi.leftJoy.getY()), leftPow) * -Math.signum(Robot.oi.leftJoy.getY());
//         double rotation = Math.pow(Math.abs(Robot.oi.rightJoy.getX()), rightPow) * Math.signum(Robot.oi.rightJoy.getX()) * OMEGA_SCALE;
       		
//         // Add a small deadzone on the joysticks
//         if (Math.abs(speed) < Math.pow(DEADZONE, leftPow)) speed = 0.0;
// 		if (Math.abs(translation) < Math.pow(DEADZONE, leftPow)) translation = 0.0;
// 		if (Math.abs(rotation) < Math.pow(DEADZONE, rightPow) * OMEGA_SCALE) rotation = 0.0;
		
		
// 		// If all of the joysticks are in the deadzone, don't update the motors
// 		// This makes side-to-side strafing much smoother
// 		if (speed == 0.0 && translation == 0.0 && rotation == 0.0) {
// 			Robot.drivetrain.setDriveLeftFront(0.0);
// 			Robot.drivetrain.setDriveLeftRear(0.0);
// 			Robot.drivetrain.setDriveRightFront(0.0);
// 			Robot.drivetrain.setDriveRightRear(0.0);
// 			return;
//    		}	

// 		//if (!Robot.oi.leftJoy.getTrigger()) {
//         	// When the Left Joystick trigger is not pressed, The robot is in Field Centric Mode.
//         	// The calculations correct the forward and strafe values for field centric attitude. 
    		
//     		// Rotate the velocity vector from the joystick by the difference between our
//     		// current orientation and the current origin heading
//     		//double originCorrection = Math.toRadians(originHeading - RobotMap.navX.getFusedHeading());
//     		// double temp = translation * Math.cos(originCorrection) - speed * Math.sin(originCorrection);
//     		// speed = speed * Math.cos(originCorrection) + translation * Math.sin(originCorrection);
//     		// translation = temp;
// 		//}
		
// 	@Override
// 	protected void initialize() {
// 		//originHeading = Robot.drivetrain.joystickdrive	
//         Robot.drivetrain.curvatureDrive(speed, translation, rotation);
// 	}
	

//     @Override
// 	protected boolean isFinished() {
//         return false;
//     }

//     @Override
// 	protected void end() {
//     }

//     @Override
// 	protected void interrupted() {
//     	end();
//     }

// }