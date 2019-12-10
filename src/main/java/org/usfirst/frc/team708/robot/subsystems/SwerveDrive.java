/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team708.robot.subsystems;

import edu.wpi.first.wpilibj.command.PIDSubsystem;
import org.usfirst.frc.team708.robot.Constants;
// import org.usfirst.frc.team708.robot.Robot;
import org.usfirst.frc.team708.robot.RobotMap;
// import org.usfirst.frc.team708.robot.commands.drivetrain.JoystickDrive;
// import org.usfirst.frc.team708.robot.commands.drivetrain.SwagDrive;
// import org.usfirst.frc.team708.robot.util.IRSensor;
// import org.usfirst.frc.team708.robot.util.UltrasonicSensor;
// import org.usfirst.frc.team708.robot.util.Math708;
// import edu.wpi.first.wpilibj.SpeedControllerGroup;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.DoubleSolenoid;
// import edu.wpi.first.wpilibj.Solenoid;

// import com.analog.adis16448.frc.ADIS16448_IMU;
// import edu.wpi.first.wpilibj.command.PIDSubsystem;
// import edu.wpi.first.wpilibj.drive.DifferentialDrive;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANEncoder;
// import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * @author Lance Hartman
 * @author Asad Bilal
 */

public class SwerveDrive extends PIDSubsystem {

  private CANSparkMax FL, FR, BL, BR;

  private TalonSRX steerFL, steerFR, steerBL, steerBR;

  public SwerveDrive(){

    super("SwerveDrive", Constants.Kp, Constants.Ki, Constants.Kd);

    FL = new CANSparkMax(RobotMap.DtDriveFrontLeftMotor, MotorType.kBrushless);  
    FR = new CANSparkMax(RobotMap.DtDriveFrontRightMotor, MotorType.kBrushless); 
    BL = new CANSparkMax(RobotMap.DtDriveBackLeftMotor, MotorType.kBrushless); 
    BR = new CANSparkMax(RobotMap.DtDriveBackRightMotor, MotorType.kBrushless); 

    steerFL = new TalonSRX(0); //add when given to us
    steerFR = new TalonSRX(0); //add when given to us
    steerBL = new TalonSRX(0); //add when given to us
    steerBR = new TalonSRX(0); //add when given to us

}

public void calculateMeasurements(double omega, double translationX, double translationY){

  double omegaL2 = omega * (Constants.WHEEL_BASE_LENGTH / 2.0);
  double omegaW2 = omega * (Constants.WHEEL_BASE_WIDTH / 2.0);

  double A = translationX - omegaL2; //Vx - omega(L/2)
  double B = translationX + omegaL2; //Vx + omega(L/2)
  double C = translationY - omegaW2; //Vy - omega(W/2)
  double D = translationY + omegaW2; //Vy + omega(W/2)

  double speedFL = speed(B, C);
  double speedFR = speed(B, D);
  double speedBL = speed(A, D);
  double speedBR = speed(A, C);

  double angleFL = angle(B, C);
  double angleFR = angle(B, C);
  double angleBL = angle(A, D);
  double angleBR = angle(A, C);

}

//gets the speed for each wheels necessary for movement
  private double speed(double val1, double val2){
    return Math.hypot(val1, val2);
  }

//gets the angle measure for each wheel
  private double angle(double val1, double val2){
    return Math.toDegrees(Math.atan2(val1, val2));  
  }

  private void setPosition(TalonSRX steer, CANSparkMax drive, double requestedAngle, double requestedSpeed){

    double currentPosition = steer.getSelectedSensorPosition(0); //getting the current encoder count
    double currentAngle = (currentPosition * 360.0 / Constants.STEER_ENCODER_COUNTS_PER_REV) % 360.0;

    //possibly move deltaDegrees here depending on logic

    if(currentAngle > 180){
      currentAngle -= 360; //if greater than 180, becomes opposite but less than 180 (-179 < x <179)
    }

    double deltaDegrees = -requestedAngle - currentAngle;

    //makes sure that deltaDegrees fits the -180 to 180 domain
    if(Math.abs(deltaDegrees) >= 180){
      deltaDegrees -= 360 * Math.signum(deltaDegrees);//signum simplifies commented code below
      /*
      double absDeltaDegrees = Math.abs(deltaDegrees) - 360; //take 360 off of deltaDegrees to set to a negative
      if(Math.abs(currentAngle) < Math.abs(requestedAngle)){
        absDeltaDegrees *= -1;
      }
      */
    }

    /*
    checks if it is efficient to move complementary to requested angle 
    (ex. requested angle = 45, new angle = -45; |45| + |-45| = 90)
    and once this value is calculated, moves wheel and reverses speed.
    NOTE this uses the new deltaDegrees from the if statement above, not from the top
    */
    if(Math.abs(deltaDegrees) > 90){
      //do above except base off of 90 degrees
      deltaDegrees -= 180 * Math.signum(deltaDegrees);
      requestedSpeed *= -1;
    }

    //got logic for targetPosition from 103's code. Make sure STEER_ENCODER_COUNTS_PER_REV is right constant
    double targetPosition = currentPosition + deltaDegrees * Constants.STEER_ENCODER_COUNTS_PER_REV / 360.0;

    //actually sets the position of the wheel
    steer.set(ControlMode.Position, targetPosition);
    drive.set(requestedSpeed); //check if speed is or is not a decimal/what it is supposed to be, decimal or not



  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  @Override
  protected double returnPIDInput() {
    // TODO Auto-generated method stub
    return 0;
  }

  @Override
  protected void usePIDOutput(double output) {
    // TODO Auto-generated method stub

  }

  public void sendToDashboard() {
		if (Constants.DEBUG) {
		}

    //all of this information is sent to the dashboard

	  }
}
