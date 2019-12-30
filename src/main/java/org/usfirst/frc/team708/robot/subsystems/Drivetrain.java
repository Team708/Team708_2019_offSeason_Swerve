package org.usfirst.frc.team708.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.usfirst.frc.team708.robot.Constants;
import org.usfirst.frc.team708.robot.RobotMap;
import org.usfirst.frc.team708.robot.commands.FieldCentricSwerveDrive;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team708.robot.subsystems.Drivetrain;

import java.util.Arrays;
import java.util.Collections;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANPIDController;

import com.revrobotics.CANSparkMax;

public class Drivetrain extends PIDSubsystem {

  
    public static CANSparkMax FL, FR, BL, BR;
    public static TalonSRX steerFL, steerFR, steerBL, steerBR;

    private static final int STATUS_FRAME_PERIOD = 20;

    private CANPIDController driveFLController, driveFRController, driveBLController, driveBRController;
    
    /**
     * @author Lance Hartman
     */
    
    public Drivetrain() {
        super("Drivetrain", Constants.driveKp, Constants.driveKi, Constants.driveKd);

        //Setup drive motors
        FL = new CANSparkMax(RobotMap.DtDriveFrontLeftMotor, MotorType.kBrushless);
        FR = new CANSparkMax(RobotMap.DtDriveFrontRightMotor, MotorType.kBrushless);
        BL = new CANSparkMax(RobotMap.DtDriveBackLeftMotor, MotorType.kBrushless);
        BR = new CANSparkMax(RobotMap.DtDriveBackRightMotor, MotorType.kBrushless);

        FL.setIdleMode(IdleMode.kCoast);
        FR.setIdleMode(IdleMode.kCoast);
        BL.setIdleMode(IdleMode.kCoast);
        BR.setIdleMode(IdleMode.kCoast);
    
        //Setup drive controller
        //just in case PID is wanted for the above motors
        driveFLController = new CANPIDController(FL);
        driveFRController = new CANPIDController(FR);
        driveBLController = new CANPIDController(BL);
        driveBRController = new CANPIDController(BR);
    
        configureDriveController(driveFLController);
        configureDriveController(driveFRController);
        configureDriveController(driveBLController);
        configureDriveController(driveBRController);
    
        //Setup steer motors
        steerFL = new TalonSRX(RobotMap.DtSteerFrontLeftMotor); // add when given to us
        steerFR = new TalonSRX(RobotMap.DtSteerFrontRightMotor); //add when given to us
        steerBL = new TalonSRX(RobotMap.DtSteerBackLeftMotor); //add when given to us
        steerBR = new TalonSRX(RobotMap.DtSteerBackRightMotor); //add when given to us

        configureSteerMotor(steerFL);
        configureSteerMotor(steerFR);
        configureSteerMotor(steerBL);
        configureSteerMotor(steerBR);

    }   

    private static void configureSteerMotor(TalonSRX steetMotor){
      steetMotor.configFactoryDefault();
      steetMotor.setInverted(Constants.STEER_MOTOR_INVERTED); //may not be necessary for any of the motors, just check to make sure
      steetMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
      steetMotor.setSensorPhase(true);
      steetMotor.configAllowableClosedloopError(0, 0, 0);
      steetMotor.configMotionAcceleration((int)(Constants.SWERVE_ROTATION_MAX_SPEED*12.5),0);
      steetMotor.configMotionCruiseVelocity((int)(Constants.SWERVE_ROTATION_MAX_SPEED),0);
      steetMotor.config_kP(0, Constants.steerKp,0);
      steetMotor.config_kI(0, Constants.steerKi,0);
      steetMotor.config_kD(0, Constants.steerKd,0);
      steetMotor.config_kF(0, 1023.0/Constants.SWERVE_ROTATION_MAX_SPEED, 0); //I don't know what this is but we need it apparently, check with P
      steetMotor.setNeutralMode(NeutralMode.Brake); //check if brake or coast
      steetMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, STATUS_FRAME_PERIOD, 0); //check
      steetMotor.set(ControlMode.MotionMagic,steerFL.getSelectedSensorPosition(0));
      if(!isRotationSensorConnected(steetMotor)){
        DriverStation.reportError(steetMotor + "rotation encoder not detected!", false);
      }
    }

    private static void configureDriveController(CANPIDController driveController){
      driveController.setP(Constants.driveKp);
      driveController.setI(Constants.driveKp);
      driveController.setD(Constants.driveKp);
      driveController.setSmartMotionMaxAccel(Constants.DRIVE_MOTOR_MAX_ACCEL*Constants.DRIVETRAIN_GEAR_RATIO,0);
    }

    private static boolean isRotationSensorConnected(TalonSRX rotationMotor){
      int pulseWidthPeriod = rotationMotor.getSensorCollection().getPulseWidthRiseToRiseUs();
      return pulseWidthPeriod != 0;
    }

    public static void calculateMeasurements(double omega, double translationX, double translationY){
    
      double omegaL2 = omega * (Constants.WHEEL_BASE_LENGTH / 2.0);
      double omegaW2 = omega * (Constants.WHEEL_BASE_WIDTH / 2.0);
    
      double A = translationX - omegaL2; //Vx - omega(L/2)
      double B = translationX + omegaL2; //Vx + omega(L/2)
      double C = translationY - omegaW2; //Vy - omega(W/2)
      double D = translationY + omegaW2; //Vy + omega(W/2)
    
      double speedFL = speed(B, D); //see speed method for explanation
      double speedFR = speed(B, C);
      double speedBL = speed(A, D);
      double speedBR = speed(A, C);
    
      double angleFL = angle(B, D); //see angle method for explanation
      double angleFR = angle(B, C);
      double angleBL = angle(A, D);
      double angleBR = angle(A, C);
    
      double maxSpeed = Collections.max(Arrays.asList(speedFL, speedBL, speedFR, speedBR, Constants.DRIVE_MOTOR_MAX_SPEED));
      
      SmartDashboard.putNumber("Angle", angleFL);

      // Set each swerve module, scaling the drive speeds by the maximum speed
      setSwerveModule(steerFL, FL, angleFL, speedFL / maxSpeed);
      setSwerveModule(steerFR, FR, angleFR, speedFR / maxSpeed);
      setSwerveModule(steerBL, BL, angleBL, speedBL / maxSpeed);
      setSwerveModule(steerBR, BR, angleBR, speedBR / maxSpeed);
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
    
        if(currentAngle > 180.0){
          currentAngle -= 360.0; //if greater than 180, becomes opposite but less than 180 (-179 < x <179)
        }
    
        double deltaDegrees = requestedAngle - currentAngle;
    
        //makes sure that deltaDegrees fits the -180 to 180 domain
        if(Math.abs(deltaDegrees) > 180.0){
          deltaDegrees -= 360.0 * Math.signum(deltaDegrees);//signum simplifies commented code below          
        }
    
        /*
        checks if it is efficient to move complementary to requested angle 
        (ex. requested angle = 45, new angle = -45; |45| + |-45| = 90)
        and once this value is calculated, moves wheel and reverses speed.
        NOTE this uses the new deltaDegrees from the if statement above, not from the top
        */
        if(Math.abs(deltaDegrees) > 90.0){
          //do above except base off of 90 degrees
          deltaDegrees -= 180.0 * Math.signum(deltaDegrees);
          requestedSpeed = -requestedSpeed;
        }
    
        //got logic for targetPosition from 103's code. Make sure STEER_ENCODER_COUNTS_PER_REV is right constant
        double targetPosition = currentPosition + (deltaDegrees * Constants.STEER_ENCODER_COUNTS_PER_REV / 360.0);
    
        //actually sets the position of the wheel
        steer.set(ControlMode.Position, targetPosition);
        drive.set(requestedSpeed); //check if speed is or is not a decimal/what it is supposed to be, decimal or not
        
        if(steer == steerFL){
          SmartDashboard.putNumber("targetPosition", targetPosition%360);		// Encoder raw count
          SmartDashboard.putNumber("currentPosition", currentPosition%360);
        }
      }

    //get Encoder values
	
    public double getSteerLFEncoder() {
      return steerFL.getSelectedSensorPosition(0);
    }
    
    public double getSteerLREncoder() {
      return steerBL.getSelectedSensorPosition(0);
    }
    
    public double getSteerRFEncoder() {
      return steerFR.getSelectedSensorPosition(0);
    }
    
    public double getSteerRREncoder() {
      return steerBR.getSelectedSensorPosition(0);
    }

    // //setting motors
    public void setDriveLeftFront(double speed){
      FL.set(speed);
    }

    public void setDriveLeftRear(double speed){
      BL.set(speed);
    }
    
    public void setDriveRightFront(double speed){
          FR.set(speed);
    }
    
    public void setDriveRightRear(double speed){
      BR.set(speed);
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

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new FieldCentricSwerveDrive());
    }
    

    /**
     * Sends data for this subsystem to the dashboard
     */
    public void sendToDashboard() {
      if (Constants.DEBUG) {
        SmartDashboard.putNumber("Sensor Position", steerFL.getSelectedSensorPosition(0));	
      }
  

        // SmartDashboard.putBoolean("Gear High", gearHigh);		//Drivetrain Gear mode
        // SmartDashboard.putBoolean("Brake", brake);					// Brake or Coast
    }
}