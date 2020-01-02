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
/**
 * Swerve drive code using WCP SS Swerve modules with 
 * NEOs/SparkMax for the drive and 775Pros/TalonSRX for steer
 * @author Lance Hartman
 * @author Ryan McCawley
 */

public class Drivetrain extends PIDSubsystem {

  
    public static CANSparkMax FL, FR, BL, BR;
    public static TalonSRX steerFL, steerFR, steerBL, steerBR;

    private static final int STATUS_FRAME_PERIOD = 20;

    private CANPIDController driveFLController, driveFRController, driveBLController, driveBRController;
    
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

        setSteerToZero(steerFR, Constants.STEER_FR_ENCODER_START_POS);
        setSteerToZero(steerFL, Constants.STEER_FL_ENCODER_START_POS);
        setSteerToZero(steerBL, Constants.STEER_BL_ENCODER_START_POS);
        setSteerToZero(steerBR, Constants.STEER_BR_ENCODER_START_POS);


    }   

    private static void configureSteerMotor(TalonSRX steerMotor){
      steerMotor.configFactoryDefault();
      steerMotor.setInverted(Constants.STEER_MOTOR_INVERTED); //may not be necessary for any of the motors, just check to make sure
      steerMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
      steerMotor.setSensorPhase(true);
      steerMotor.configAllowableClosedloopError(0, 0, 0);
      steerMotor.configMotionAcceleration((int)(Constants.SWERVE_ROTATION_MAX_SPEED*12.5),0);
      steerMotor.configMotionCruiseVelocity((int)(Constants.SWERVE_ROTATION_MAX_SPEED),0);
      steerMotor.config_kP(0,Constants.steerKp,0);
      steerMotor.config_kI(0,Constants.steerKi,0);
      steerMotor.config_kD(0,Constants.steerKd,0);
      steerMotor.config_kF(0,1023.0/Constants.SWERVE_ROTATION_MAX_SPEED,0); //I don't know what this is but we need it apparently, check with P
      steerMotor.setNeutralMode(NeutralMode.Brake); //check if brake or coast
      steerMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0,STATUS_FRAME_PERIOD,0); //check
      if(!isRotationSensorConnected(steerMotor)){
        DriverStation.reportError(steerMotor + "rotation encoder not detected!", false);
      }
    }

    private static void configureDriveController(CANPIDController driveController){
      driveController.setP(Constants.driveKp);
      driveController.setI(Constants.driveKp);
      driveController.setD(Constants.driveKp);
      driveController.setSmartMotionMaxAccel(Constants.DRIVE_MOTOR_MAX_ACCEL*Constants.DRIVETRAIN_GEAR_RATIO,0);
      driveController.setSmartMotionMaxVelocity(Constants.DRIVE_MOTOR_MAX_POWER, 0);
    }

    private static boolean isRotationSensorConnected(TalonSRX rotationMotor){
      int pulseWidthPeriod = rotationMotor.getSensorCollection().getPulseWidthRiseToRiseUs();
      return pulseWidthPeriod != 0;
    }
    /**
	   * Sets the swerve drive local linear and angular velocity.
     * @param translationX movement side to side. Value should be between -1.0 and 1.0.
     * @param translationY movement front to back. Value should be between -1.0 and 1.0.
     * @param rotation rotation about robot center. Value should be between -1.0 and 1.0.
	   */
    public static void set(double translationX, double translationY, double rotation){
    
      double rotationL2 = rotation * (Constants.WHEEL_BASE_LENGTH / 2.0);
      double rotationW2 = rotation * (Constants.WHEEL_BASE_WIDTH / 2.0);
    
      double A = translationX - rotationL2; //Vx - rotation(L/2)
      double B = translationX + rotationL2; //Vx + rotation(L/2)
      double C = translationY - rotationW2; //Vy - rotation(W/2)
      double D = translationY + rotationW2; //Vy + rotation(W/2)
    
      double speedFR = speed(B, C); //see speed method for explanation
      double speedFL = speed(B, D);
      double speedBL = speed(A, D);
      double speedBR = speed(A, C);
    
      double angleFR = angle(B, C);
      double angleFL = angle(B, D); //see angle method for explanation
      double angleBL = angle(A, D);
      double angleBR = angle(A, C);
    
      double maxSpeed = Collections.max(Arrays.asList(speedFL, speedBL, speedFR, speedBR, Constants.DRIVE_MOTOR_MAX_POWER));
      
      SmartDashboard.putNumber("Angle", angleFL);

      // Set each swerve module, scaling the drive speeds by the maximum speed
      setSwerveModule(steerFL, FL, angleFL, speedFL / maxSpeed, Constants.STEER_FL_ENCODER_START_POS);
      setSwerveModule(steerFR, FR, angleFR, speedFR / maxSpeed, Constants.STEER_FR_ENCODER_START_POS);
      setSwerveModule(steerBL, BL, angleBL, speedBL / maxSpeed, Constants.STEER_BL_ENCODER_START_POS);
      setSwerveModule(steerBR, BR, angleBR, speedBR / maxSpeed, Constants.STEER_BR_ENCODER_START_POS);
    }
    
    /**
     * Gets the speed for each wheels along the direction of travel
     * @param x value of motion along X direction. 
     * @param y value of motion along Y direction.
     * @return sqrt(<i>x</i><sup>2</sup>&nbsp;+<i>y</i><sup>2</sup>)
     * without intermediate overflow or underflow
     */
      private static double speed(double x, double y){
        return Math.hypot(x, y);   //relative to pythagorean theorem, x = a, y = b, return value = c
      }

    /**
     * Gets the angle for each wheels along the direction of travel
     * @param x value of rotational torque along X direction. 
     * @param y value of rotational torque along Y direction.
     * @return <i>theta</i> in degrees
     */
      private static double angle(double x, double y){
        return Math.toDegrees(Math.atan2(x, y)); //relative to theta, x = opp., y = adj., outputs degree of theta
      }

      /**
       * Sets the speed and direction of each swerve module.
       * @param steer module steer motor controller (TalonSRX)
       * @param drive module drive motor controller (SparkMax)
       * @param requestedAngle module angle to be set. Value should be between -180.0 and 180.0.
       * @param requestedSpeed  module speed to be set. Value should be between -1.0 and 1.0.
       */
      private static void setSwerveModule(TalonSRX steer, CANSparkMax drive, double requestedAngle, double requestedSpeed, double angleOffset){
    
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
          SmartDashboard.putNumber("targetPosition", targetPosition%4096);		// Encoder raw count
          SmartDashboard.putNumber("currentPosition", currentPosition%4096);
        }
      }

    //get Encoder values
    /**
	   * Get the front right steer encoder position (in raw sensor units).
	   * @return Position of selected sensor (in raw sensor units).
	   */
    public double getSteerFREncoder() {
      return steerFR.getSelectedSensorPosition(0);
    }

		/**
	   * Get the front left steer encoder position (in raw sensor units).
	   * @return Position of selected sensor (in raw sensor units).
	   */
    public double getSteerFLEncoder() {
      return steerFL.getSelectedSensorPosition(0);
    }
 
		/**
	   * Get the back left steer encoder position (in raw sensor units).
	   * @return Position of selected sensor (in raw sensor units).
	   */
    public double getSteerBLEncoder() {
      return steerBL.getSelectedSensorPosition(0);
    }
    
    /**
	   * Get the back right steer encoder position (in raw sensor units).
	   * @return Position of selected sensor (in raw sensor units).
	   */
    public double getSteerBREncoder() {
      return steerBR.getSelectedSensorPosition(0);
    }

    //setting motors
    /**
     * Interface for setting the speed of the front left drive motor.
     *
     * @param speed The speed to set. Value should be between -1.0 and 1.0.
     */
    public void setDriveFL(double speed){
      FL.set(speed);
    }

	/**
     * Interface for setting the speed of the back left drive motor.
	 *
	 * @param speed The speed to set. Value should be between -1.0 and 1.0.
	 */
    public void setDriveBL(double speed){
      BL.set(speed);
    }
    
    /**
     * Interface for setting the speed of the front right drive motor.
     *
     * @param speed The speed to set. Value should be between -1.0 and 1.0.
     */
    public void setDriveFR(double speed){
      FR.set(speed);
    }
    
    /**
     * Interface for setting the speed of the back right drive motor.
     *
     * @param speed The speed to set. Value should be between -1.0 and 1.0.
     */
    public void setDriveBR(double speed){
      BR.set(speed);
    }

    public void setSteerToZero(TalonSRX steerMotor, double angleOffset){
      steerMotor.set(ControlMode.MotionMagic,steerFL.getSelectedSensorPosition(0) - angleOffset);
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
    }
}