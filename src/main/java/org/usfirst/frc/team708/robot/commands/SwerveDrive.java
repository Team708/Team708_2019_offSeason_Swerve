/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team708.robot.commands;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDBase;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import org.usfirst.frc.team708.robot.Constants;
import org.usfirst.frc.team708.robot.Robot;
// import org.usfirst.frc.team708.robot.Robot;
import org.usfirst.frc.team708.robot.RobotMap;
import org.usfirst.frc.team708.robot.commands.drivetrain.JoystickDrive;
// import org.usfirst.frc.team708.robot.commands.drivetrain.JoystickDrive;
// import org.usfirst.frc.team708.robot.commands.drivetrain.SwagDrive;
// import org.usfirst.frc.team708.robot.util.IRSensor;
// import org.usfirst.frc.team708.robot.util.UltrasonicSensor;
// import org.usfirst.frc.team708.robot.util.Math708;
// import edu.wpi.first.wpilibj.SpeedControllerGroup;
import org.usfirst.frc.team708.robot.subsystems.Drivetrain;

import java.util.Arrays;
import java.util.Collections;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANPIDController;

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
 */

public class SwerveDrive extends Command{

  private CANSparkMax FL, FR, BL, BR;
  private TalonSRX steerFL, steerFR, steerBL, steerBR;

  private CANPIDController driveFLController, driveFRController, driveBLController, driveBRController;

  // private TalonSRX steerFL, steerFR, steerBL, steerBR;

  JoystickDrive joystickDrive;

  public SwerveDrive() {
    joystickDrive = new JoystickDrive();
    requires(Robot.drivetrain);

}

public static void calculateMeasurements(double omega, double translationX, double translationY){

  double omegaL2 = omega * (Constants.WHEEL_BASE_LENGTH / 2.0);
  double omegaW2 = omega * (Constants.WHEEL_BASE_WIDTH / 2.0);

  double A = translationX - omegaL2; //Vx - omega(L/2)
  double B = translationX + omegaL2; //Vx + omega(L/2)
  double C = translationY - omegaW2; //Vy - omega(W/2)
  double D = translationY + omegaW2; //Vy + omega(W/2)

  double speedFL = speed(B, C); //see speed method for explanation
  double speedFR = speed(B, D);
  double speedBL = speed(A, D);
  double speedBR = speed(A, C);

  double angleFL = angle(B, D) + 90; //see angle method for explanation
  double angleFR = angle(B, C) + 90;
  double angleBL = angle(A, D) - 90;
  double angleBR = angle(A, C) - 90;

  double maxSpeed = Collections.max(Arrays.asList(speedFL, speedBL, speedFR, speedBR, 1.0));

  // Set each swerve module, scaling the drive speeds by the maximum speed
  setSwerveModule(Drivetrain.steerFL, Drivetrain.FL, angleFL, speedFL / maxSpeed);
  setSwerveModule(Drivetrain.steerFR, Drivetrain.FR, angleFR, speedFR / maxSpeed);
  setSwerveModule(Drivetrain.steerBL, Drivetrain.BL, angleBL, speedBL / maxSpeed);
  setSwerveModule(Drivetrain.steerBR, Drivetrain.BR, angleBR, speedBR / maxSpeed);

  
}

//gets the speed for each wheels necessary for movement
  private static double speed(double val1, double val2){
    return Math.hypot(val1, val2); //relative to pythagorean theorem, val1 = a, val2 = b, return value = c
  }

//gets the angle measure for each wheel
  private static double angle(double val1, double val2){
    return Math.toDegrees(Math.atan2(val1, val2)); //relative to theta, val1 = opp., val2 = adj., outputs degree of theta
  }

  private static void setSwerveModule(TalonSRX steer, CANSparkMax drive, double requestedAngle, double requestedSpeed){

    double currentPosition = steer.getSelectedSensorPosition(0); //getting the current encoder count
    double currentAngle = (currentPosition * 360.0 / Constants.STEER_ENCODER_COUNTS_PER_REV) % 360.0;

    System.out.println(currentPosition);

    if(currentAngle > 180){
      currentAngle -= 360; //if greater than 180, becomes opposite but less than 180 (-179 < x <179)
    }

    double deltaDegrees = -requestedAngle - currentAngle;

    //makes sure that deltaDegrees fits the -180 to 180 domain
    if(Math.abs(deltaDegrees) >= 180){
      deltaDegrees -= 360 * Math.signum(deltaDegrees);//signum simplifies commented code below
      /*
      double absDeltaDegrees = Math.abs(deltaDegrees) - 360; //take 360 off of deltaDegrees to set it to a negative angle
      if(Math.abs(currentAngle) < Math.abs(requestedAngle)){
        absDeltaDegrees *= -1;
      }
      steer.set(ControlMode.Position, deltaDegrees);
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


    public void resetEncoder() {
  


    }

    @Override
    protected boolean isFinished() {
      // TODO Auto-generated method stub
      return false;
    }

  public void sendToDashboard() {
		if (Constants.DEBUG) {
		}

    //all of this information is sent to the dashboard

    }

  }