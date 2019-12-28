package org.usfirst.frc.team708.robot.subsystems;

import java.util.Arrays;
import java.util.Collections;
import org.usfirst.frc.team708.robot.Constants;
import org.usfirst.frc.team708.robot.Robot;
import org.usfirst.frc.team708.robot.RobotMap;
import org.usfirst.frc.team708.robot.commands.JoystickDrive;

//import org.usfirst.frc.team708.robot.util.IRSensor;
//import org.usfirst.frc.team708.robot.util.UltrasonicSensor;
// import org.usfirst.frc.team708.robot.util.Math708;
// import edu.wpi.first.wpilibj.SpeedControllerGroup;

//import edu.wpi.first.wpilibj.DigitalInput;
//import edu.wpi.first.wpilibj.Solenoid;

// import com.analog.adis16448.frc.ADIS16448_IMU;
// import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.command.Subsystem;
// import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANEncoder;
// import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.*;

public class Drivetrain extends Subsystem {

	private CANSparkMax driveLeftFrontSpark, driveLeftRearSpark, driveRightFrontSpark, driveRightRearSpark;
	private TalonSRX leftFrontTalon, leftRearTalon, rightFrontTalon, rightRearTalon;
	// private CANEncoder encoderLeftFront, encoderLeftRear, encoderRightFront, encoderRightRear;
	
	// Variables specific for drivetrain PID loop
	// private double moveSpeed = 0.0;
	// private double pidOutput = 0.0;
	
	// private double gearratio;
	
	// private DifferentialDrive drivetrain;						// FRC provided drivetrain class
	// private double revPerInch;
	// private boolean gearHigh;
	// private ADIS16448_IMU gyro;
	
	//private Solenoid gearShifter;
	
	// private IRSensor drivetrainIRSensor;					// IR Sensor for <=25inches
	// private UltrasonicSensor drivetrainUltrasonicSensor;	// Sonar used for <=21feet
	// private DigitalInput lineSensor;
	
	// private boolean brake = true;	
	// private boolean usePID = false;
	// public boolean tilting = false;

	// public boolean runningCG = false;
	// public boolean runningAuto = false;

	public static final double WHEEL_BASE_LENGTH = 24; 
  	public static final double WHEEL_BASE_WIDTH = 24;
  //Set the ENCODER_COUNT_PER_ROTATION to the value for the pivot encoder 
  //MA3 encoders = 1024, Versaplanetary (SRXEncoders) = 4096
  //Talon SRX decode 4X only, relating to 4096 CPR (counts per rev) for the CTRE
  // mag encoder 
	public static final double ENCODER_COUNT_PER_ROTATION = 4096.0;

	public static final double STEER_DEGREES_PER_COUNT = 360.0 / ENCODER_COUNT_PER_ROTATION;
	public static final double DEADZONE = 0.08;

	private static final double STEER_P = 2.0, STEER_I = 0.0, STEER_D = 20.0;
  private static final int STATUS_FRAME_PERIOD = 20;
  private static final double RAMP_RATE = 0.5;

	
	public Drivetrain() {
		// Passes variables from this class into the superclass constructor
		//super("Drivetrain", Constants.Kp, Constants.Ki, Constants.Kd);
    	
		
		// SpeedControllerGroup leftMotors = new SpeedControllerGroup(driveLeftFrontSpark, driveLeftRearSpark);
		// SpeedControllerGroup rightMotors = new SpeedControllerGroup(driveRightFrontSpark, driveRightRearSpark);
		
		// drivetrain = new DifferentialDrive(leftMotors, rightMotors);	// Initializes drivetrain class
		
		// gyro = new ADIS16448_IMU();
		// gyro.reset();
		
		// encoderLeftFront = new CANEncoder(driveLeftFrontSpark);
		// encoderLeftRear = new CANEncoder(driveLeftRearSpark);
		// encoderRightFront = new CANEncoder(driveRightFrontSpark);
		// encoderRightRear = new CANEncoder(driveRightRearSpark);

		// Initializes the sensors
		// resetEncoder();

		// lineSensor = new DigitalInput(RobotMap.lineSensor);
		// gearShifter	=	new Solenoid(RobotMap.driveShift);

	leftFrontTalon = new TalonSRX(RobotMap.LEFT_FRONT_TALON);
    leftFrontTalon.configFactoryDefault();
    // use this for MA3 encoders
    //steerLeftFront.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);
    //Use this for VersaPlanetary encoders
    leftFrontTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
    //If the pivots turn inward forming an X pattern change the inversion of the motors
    leftFrontTalon.setInverted(true);
    leftFrontTalon.config_kP(0, STEER_P, 0);
    leftFrontTalon.config_kI(0, STEER_I, 0);
    leftFrontTalon.config_kD(0, STEER_D, 0);
    leftFrontTalon.config_IntegralZone(0, 100, 0);
    leftFrontTalon.configAllowableClosedloopError(0, 5, 0);
    leftFrontTalon.setNeutralMode(NeutralMode.Brake);
    leftFrontTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, STATUS_FRAME_PERIOD, 0);

	leftRearTalon = new TalonSRX(RobotMap.LEFT_REAR_TALON);
    leftRearTalon.configFactoryDefault();
    // use this for MA3 encoders
    //steerLeftRear.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);
    //Use this for VersaPlanetary encoders
    leftRearTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
    //If the pivots turn inward forming an X pattern change the inversion of the motors
		leftRearTalon.setInverted(true);
    leftRearTalon.config_kP(0, STEER_P, 0);
    leftRearTalon.config_kI(0, STEER_I, 0);
    leftRearTalon.config_kD(0, STEER_D, 0);
    leftRearTalon.config_IntegralZone(0, 100, 0);
    leftRearTalon.configAllowableClosedloopError(0, 5, 0);
    leftRearTalon.setNeutralMode(NeutralMode.Brake);
    leftRearTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, STATUS_FRAME_PERIOD, 0);

	rightFrontTalon = new TalonSRX(RobotMap.RIGHT_FRONT_TALON);
    rightFrontTalon.configFactoryDefault();
    // use this for MA3 encoders
    //steerRightFront.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);
    //Use this for VersaPlanetary encoders
    rightFrontTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
    //If the pivots turn inward forming an X pattern change the inversion of the motors
    rightFrontTalon.setInverted(true);
    rightFrontTalon.config_kP(0, STEER_P, 0);
    rightFrontTalon.config_kI(0, STEER_I, 0);
    rightFrontTalon.config_kD(0, STEER_D, 0);
    rightFrontTalon.config_IntegralZone(0, 100, 0);
    rightFrontTalon.configAllowableClosedloopError(0, 5, 0);
    rightFrontTalon.setNeutralMode(NeutralMode.Brake);
    rightFrontTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, STATUS_FRAME_PERIOD, 0);


	rightRearTalon = new TalonSRX(RobotMap.RIGHT_REAR_TALON);
    rightRearTalon.configFactoryDefault();
    // use this for MA3 encoders
    //steerRightRear.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);
    //Use this for VersaPlanetary encoders
	rightRearTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
    //If the pivots turn inward forming an X pattern change the inversion of the motors
    rightRearTalon.setInverted(true);
    rightRearTalon.config_kP(0, STEER_P, 0);
    rightRearTalon.config_kI(0, STEER_I, 0);
    rightRearTalon.config_kD(0, STEER_D, 0);
    rightRearTalon.config_IntegralZone(0, 100, 0);
    rightRearTalon.configAllowableClosedloopError(0, 5, 0);
    rightRearTalon.setNeutralMode(NeutralMode.Brake);
	rightRearTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, STATUS_FRAME_PERIOD, 0);

	driveLeftFrontSpark = new CANSparkMax(RobotMap.DRIVE_LEFT_REAR_SPARK, MotorType.kBrushless);
	driveLeftFrontSpark.restoreFactoryDefaults();
    driveLeftFrontSpark.setOpenLoopRampRate(RAMP_RATE);
    //If you calibrate your drive encoder to zero with the pivot facing backwards the drive will
    //run backwards. If this occurs you will need to invert the drive (same for each spark)
    driveLeftFrontSpark.setInverted(false);
        
	driveLeftRearSpark = new CANSparkMax(RobotMap.DRIVE_LEFT_REAR_SPARK, MotorType.kBrushless);
	driveLeftRearSpark.restoreFactoryDefaults();
    driveLeftRearSpark.setOpenLoopRampRate(RAMP_RATE); 
    driveLeftRearSpark.setInverted(false);
  
	driveRightFrontSpark = new CANSparkMax(RobotMap.DRIVE_RIGHT_FRONT_SPARK, MotorType.kBrushless);
	driveRightFrontSpark.restoreFactoryDefaults();
    driveRightFrontSpark.setOpenLoopRampRate(RAMP_RATE); 
	driveRightFrontSpark.setInverted(false);
  
	driveRightRearSpark = new CANSparkMax(RobotMap.DRIVE_RIGHT_REAR_SPARK, MotorType.kBrushless);
	driveRightRearSpark.restoreFactoryDefaults();
    driveRightRearSpark.setOpenLoopRampRate(RAMP_RATE); 
    driveRightRearSpark.setInverted(false);
	}

	public void swerveDrive(double translationX, double translationY, double rotation) {
    double omegaL2 = rotation * (WHEEL_BASE_LENGTH / 2.0);
    double omegaW2 = rotation * (WHEEL_BASE_WIDTH / 2.0);
    
    // Compute the constants used later for calculating speeds and angles
    double A = translationX - omegaL2;
    double B = translationX + omegaL2;
    double C = translationY - omegaW2;
    double D = translationY + omegaW2;
    
    // Compute the drive motor speeds
    double speedLF = speed(B, D);
    double speedLR = speed(A, D);
    double speedRF = speed(B, C);
    double speedRR = speed(A, C);
    
		// ... and angles for the steering motors 
		// When drives are calibrated for zero position on encoders they can be at 90 degrees
		// to the front of the robot. Subtract and add 90 degrees to steering calculation to offset
    // for initial position/calibration of drives if the drive zero position faces the side of
    // the robot.

	double angleLF = angle(B, D) + 90;
    double angleLR = angle(A, D) - 90;
    double angleRF = angle(B, C) + 90;
    double angleRR = angle(A, C) - 90;
    // Compute the maximum speed so that we can scale all the speeds to the range [0, 1]
    double maxSpeed = Collections.max(Arrays.asList(speedLF, speedLR, speedRF, speedRR, 1.0));

    // Set each swerve module, scaling the drive speeds by the maximum speed
    setSwerveModule(leftFrontTalon, driveLeftFrontSpark, angleLF, speedLF / maxSpeed);
    setSwerveModule(leftRearTalon, driveLeftRearSpark, angleLR, speedLR / maxSpeed);
    setSwerveModule(rightFrontTalon, driveRightFrontSpark, angleRF, speedRF / maxSpeed);
	setSwerveModule(rightRearTalon, driveRightRearSpark, angleRR, speedRR / maxSpeed);
	}
	
	private double speed(double val1, double val2){
	return Math.hypot(val1, val2);
	}	
  
	private double angle(double val1, double val2){
	return Math.toDegrees(Math.atan2(val1, val2));
	}
	
	private void setSwerveModule(TalonSRX steer, CANSparkMax drive, double angle, double speed) {
    double currentPosition = steer.getSelectedSensorPosition(0);
    double currentAngle = (currentPosition * 360.0 / ENCODER_COUNT_PER_ROTATION) % 360.0;
    // The angle from the encoder is in the range [0, 360], but the swerve computations
    // return angles in the range [-180, 180], so transform the encoder angle to this range
    if (currentAngle > 180.0) {
      currentAngle -= 360.0;
    }
    // TO DO: Properly invert the steering motors so this isn't necessary
    // This is because the steering encoders are inverted
    double targetAngle = -angle;
    double deltaDegrees = targetAngle - currentAngle;
    // If we need to turn more than 180 degrees, it's faster to turn in the opposite direction
    if (Math.abs(deltaDegrees) > 180.0) {
      deltaDegrees -= 360.0 * Math.signum(deltaDegrees);
    }
    // If we need to turn more than 90 degrees, we can reverse the wheel direction instead and
		// only rotate by the complement
		
		//if (Math.abs(speed) <= MAX_SPEED){
    if (Math.abs(deltaDegrees) > 90.0) {
      deltaDegrees -= 180.0 * Math.signum(deltaDegrees);
      speed = -speed;
    }
		//}
		

		double targetPosition = currentPosition + deltaDegrees * ENCODER_COUNT_PER_ROTATION / 360.0;
		steer.set(ControlMode.Position, targetPosition);
		drive.set(speed);

	}

	//get Encoder values
	
	// public double getSteerLFEncoder() {
	// 	return leftFrontTalon.getSelectedSensorPosition(0);
	// }
	
	// public double getSteerLREncoder() {
	// 	return leftRearTalon.getSelectedSensorPosition(0);
	// }
	
	// public double getSteerRFEncoder() {
	// 	return rightFrontTalon.getSelectedSensorPosition(0);
	// }
	
	// public double getSteerRREncoder() {
	// 	return rightRearTalon.getSelectedSensorPosition(0);
	// }

		
	/**
	 * Initializes the default command for this subsystem
	 */
	@Override
	public void initDefaultCommand() {
    setDefaultCommand(new JoystickDrive());
    // setDefaultCommand(new SwagDrive());
  }
  
	/**
	 * Drives the drivetrain using a forward-backward value and a rotation value
	 * @param move
	 * @param rotate
	 */
	// public void haloDrive(double move, double rotate, boolean usePID)
	// {
	// 	move = move * Constants.DRIVE_MOTOR_MAX_SPEED;
	// 	rotate = rotate * Constants.ROTATE_MOTOR_MAX_SPEED;    	
		// if (usePID)
		// {
		// 	if (rotate == 0.0 && move > 0.0)
		// 	{
		// 		// Enables the PID controller if it is not already
		// 		if (!getPIDController().isEnabled()) 
		// 		{
		// 			getPIDController().setPID(Constants.KpForward, Constants.KiForward, Constants.KdForward);
		// 			getPIDController().reset();
		// 			gyro.reset();
		// 			enable();
		// 			gyro.reset();
		// 		}
		// 		// Sets the forward move speed to the move parameter
		// 		moveSpeed = move;
		// 	} 
		// 	else if (rotate == 0.0 && move < 0.0)
		// 	{
		// 		// Enables the PID controller if it is not already
		// 		if (!getPIDController().isEnabled())
		// 		{
		// 			getPIDController().setPID(Constants.KpBackward, Constants.KiBackward, Constants.KdBackward);
		// 			getPIDController().reset();
		// 			gyro.reset();
		// 			enable();
		// 			gyro.reset();
		// 		}
		// 		// Sets the forward move speed to the move parameter
		// 		moveSpeed = move;
		// 	} else 
		// 	{
		// 		// Disables the PID controller if it enabled so the drivetrain can move freely
		// 		if (getPIDController().isEnabled()) 
		// 		{
		// 			getPIDController().reset();
		// 		}
		// 		drivetrain.arcadeDrive(move, rotate);
		// 	}
		// } 
		// else 
		// {
		// 	// Disables the PID controller if it enabled so the drivetrain can move freely
		// 	if (getPIDController().isEnabled()) 
		// 	{
		// 		getPIDController().reset();
		// 	}
		//	drivetrain.arcadeDrive(move, rotate);
		// }
  //}
	
	// public void haloDrive(double move, double rotate) {
	// 	haloDrive(move, rotate, this.usePID);
	// }
		
	/**
	 * Drive the drivetrain using curvature drive
	 * @param xSpeed
	 * @param zRotation
	 * @param isQuickTurn
	 */
	// public void curvatureDrive(double xSpeed, double zRotation, boolean isQuickTurn) {
	// 	drivetrain.curvatureDrive(xSpeed, zRotation, isQuickTurn);
	// }
    
	public void stop() {
		// driveLeftFrontSpark.set(Constants.DRIVE_MOTOR_OFF);
		// driveLeftRearSpark.set(Constants.DRIVE_MOTOR_OFF);
		// driveRightFrontSpark.set(Constants.DRIVE_MOTOR_OFF);
		// driveRightRearSpark.set(Constants.DRIVE_MOTOR_OFF);
	}

	// public double getAngle() {
	// 		return  Math708.round(gyro.getAngleZ(),0);
	// }
	
	// public boolean isTiltingLeft() {
	// 	if (gyro.getRoll() >= Constants.ROLL_MAX) {
	// 		tilting = true;
	// 		return true;
	// 	}
	// 	else {
	// 		tilting = false;
	// 		return false;	
	// 	}	
	// }

	// public boolean isTiltingRight() {
	// 	if (gyro.getRoll() <= -Constants.ROLL_MAX) {
	// 		tilting = true;
	// 		return true;
	// 	}
	// 	else {
	// 		tilting = false;
	// 		return false;	
	// 	}	
	// }

// 	public void resetGyro() {
// 		gyro.reset();
//   }
    
// 	public double rotateByGyro(double targetAngle, double tolerance) {
// 		double difference = getAngle() - targetAngle;
// 		if (Math708.isWithinThreshold(gyro.getAngle(), targetAngle, tolerance)) {
// 			difference = 0.0;
// 		}		
// 		return difference / targetAngle;
// 	}
	
// 	public double getIRDistance() {
// 		return drivetrainIRSensor.getAverageDistance();
// 	}
	
// 	public double getSonarDistance() {
// 		return drivetrainUltrasonicSensor.getClippedAverageDistance();
// 	}
    
	/**
	 * Returns the move speed of the robot needed to get to a certain IR distance reading.
	 * This assumes that the IR sensor is in the front of the robot.
	 * @param targetDistance
	 * @return
	 */
	// public double moveByIR(double targetDistance, double minSpeed, double maxSpeed, double tolerance) {
	// 	double current_location = getIRDistance();
	// 	double value = Math708.getClippedPercentError(current_location, targetDistance, minSpeed, maxSpeed);
	// 	if (value <= 0.0 || ((Math.abs(current_location - targetDistance)) <= tolerance)) {			
	// 		return 0.0;
	// 	}
	// 	return value;
	// }

	/**
	 * Returns the move speed of the robot needed to get to a certain Sonar distance reading.
	 * This assumes that the Sonar sensor is in the front of the robot.
	 * @param targetDistance
	 * @return
	 */
	// public double moveByUltrasonic(double targetDistance, double minSpeed, double maxSpeed, double tolerance) {
	// 	double value = Math708.getClippedPercentError(getSonarDistance(), targetDistance, minSpeed, maxSpeed);
	// 	if (value <= 0.0 || ((Math.abs(getSonarDistance() - targetDistance)) <= tolerance)) {
	// 		return 0.0;
	// 	}
	// 	return value;
	// }
    
	// public void toggleBrakeMode() {
	// 	brake = !brake;
	// 	if (brake) {
	// 		leftMaster.setIdleMode(IdleMode.kBrake);
	// 		leftSlave1.setIdleMode(IdleMode.kBrake);
	// 		rightMaster.setIdleMode(IdleMode.kBrake);
	// 		rightSlave1.setIdleMode(IdleMode.kBrake);
	// 	} 
	// 	else {
	// 		leftMaster.setIdleMode(IdleMode.kCoast);
	// 		leftSlave1.setIdleMode(IdleMode.kCoast);
	// 		rightMaster.setIdleMode(IdleMode.kCoast);
	// 		rightSlave1.setIdleMode(IdleMode.kCoast);
	// 	}
	// }
    
// 	public void setBrakeMode(boolean setBrake) {
// 		brake = setBrake;
// 		if (brake) {
// 			driveLeftFrontSpark.setIdleMode(IdleMode.kBrake);
// 			driveLeftRearSpark.setIdleMode(IdleMode.kBrake);
// 			driveRightFrontSpark.setIdleMode(IdleMode.kBrake);
// 			driveRightRearSpark.setIdleMode(IdleMode.kBrake);
// 			leftFrontTalon.setNeutralMode(NeutralMode.Brake);
// 			leftRearTalon.setNeutralMode(NeutralMode.Brake);
// 			rightFrontTalon.setNeutralMode(NeutralMode.Brake);
// 			rightRearTalon.setNeutralMode(NeutralMode.Brake);
// 		} 
// 		else {
// 			driveLeftFrontSpark.setIdleMode(IdleMode.kCoast);
// 			driveLeftRearSpark.setIdleMode(IdleMode.kCoast);
// 			driveRightFrontSpark.setIdleMode(IdleMode.kCoast);
// 			driveRightRearSpark.setIdleMode(IdleMode.kCoast);
// 			leftFrontTalon.setNeutralMode(NeutralMode.Coast);
// 			leftRearTalon.setNeutralMode(NeutralMode.Coast);
// 			rightFrontTalon.setNeutralMode(NeutralMode.Coast);
// 			rightRearTalon.setNeutralMode(NeutralMode.Coast);
// ;		}
// 	}

	// public void shiftGearHigh() {
	// 	gearHigh = true;
	// 	gearShifter.set(gearHigh);
	// 	gearratio = Constants.DRIVETRAIN_GEAR_RATIO_HIGH;
	// 	revPerInch = (Math.PI*Constants.DRIVETRAIN_WHEEL_DIAMETER)/gearratio;
	// }
	
	// public void shiftGearlow() {
	// 	gearHigh = false;
	// 	gearShifter.set(gearHigh);		
	// 	gearratio = Constants.DRIVETRAIN_GEAR_RATIO_LOW;
	// 	revPerInch = (Math.PI*Constants.DRIVETRAIN_WHEEL_DIAMETER)/gearratio;
	// }
		
	/**
	 * 
	 * @return Distance traveled since last encoder reset
	 */
	// public double getEncoderDistanceLeftFrontS() {
	// 	return encoderLeftFront.getPosition() * revPerInch;
	// }
		
	// public double getEncoderDistanceLeftRearS() {
	// 	return encoderLeftRear.getPosition() * revPerInch;
	// }
	
	// public double getEncoderDistanceRightFrontS() {
	// 	return encoderRightFront.getPosition() * revPerInch;
	// }
		
	// public double getEncoderDistanceRightRearS() {
	// 	return encoderRightRear.getPosition() * revPerInch;
	// }

	// public double getEncoderDistanceLeftFrontT() {
	// 	return encoderLeftFront.getPosition() * revPerInch;
	// }
			
	// public double getEncoderDistanceLeftRearT() {
	// 	return encoderLeftRear.getPosition() * revPerInch;
	// }
		
	// public double getEncoderDistanceRightFrontT() {
	// 	return encoderRightFront.getPosition() * revPerInch;
	// }
			
	// public double getEncoderDistanceRightRearT() {
	// 	return encoderRightRear.getPosition() * revPerInch;
	// }
	
	/**
	 * Resets the encoder to 0.0
	 */
	// public void resetEncoder() {
	// 	encoderLeftFront.setPosition(0.0);
	// 	encoderLeftRear.setPosition(0.0);
	// 	encoderRightFront.setPosition(0.0);
	// 	encoderRightRear.setPosition(0.0);
	// }

	// public void setDriveLeftFront(double speed){
	// 	driveLeftFrontSpark.set(speed);
	// }

	// public void setDriveLeftRear(double speed){
	// 	driveLeftRearSpark.set(speed);
	// }
	
	// public void setDriveRightFront(double speed){
	// 	driveRightFrontSpark.set(speed);
	// }
	
	// public void setDriveRightRear(double speed){
	// 	driveRightRearSpark.set(speed);
	// }
	
	// public boolean isOnLine() {
	// 	return !lineSensor.get();
	// }
	
	/**
	 * Returns a process variable to the PIDSubsystem for correction
	 */
//	@Override
// 	protected double returnPIDInput() {
//     return getAngle();
//   }
    
	/**
	 * Performs actions using the robot to correct for any error using the outputed value
	 */
// 	@Override
// 	protected void usePIDOutput(double output) {
// 		pidOutput = output;
// 		drivetrain.arcadeDrive(moveSpeed, -output);
//   }     
    
	/**
	 * Sends data for this subsystem to the dashboard
	 */
	public void sendToDashboard() {
		if (Constants.DEBUG) {
		}
			// SmartDashboard.putNumber("DT Encoder Left Rev", encoderLeftFront.getPosition());		// Encoder raw count
			// SmartDashboard.putNumber("DT Encoder Right Rev", encoderLeftRear.getPosition());		// Encoder raw count
			// SmartDashboard.putNumber("DT Encoder Left Rev", encoderRightFront.getPosition());		// Encoder raw count
			// SmartDashboard.putNumber("DT Encoder Right Rev", encoderRightRear.getPosition());		// Encoder raw count
			// SmartDashboard.putNumber("DT Encoder Left Inches", getEncoderDistanceLeftFrontS());		// Encoder inches
			// SmartDashboard.putNumber("DT Encoder Right Inches", getEncoderDistanceLeftRearS());		// Encoder inches
			// SmartDashboard.putNumber("DT Encoder Left Inches", getEncoderDistanceRightFrontS());		// Encoder inches
			// SmartDashboard.putNumber("DT Encoder Right Inches", getEncoderDistanceRightRearS());		// Encoder inches
			// SmartDashboard.putNumber("DT Encoder Left Inches", getEncoderDistanceLeftFrontT());		// Encoder inches
			// SmartDashboard.putNumber("DT Encoder Right Inches", getEncoderDistanceLeftRearT());		// Encoder inches
			// SmartDashboard.putNumber("DT Encoder Left Inches", getEncoderDistanceRightFrontT());		// Encoder inches
			// SmartDashboard.putNumber("DT Encoder Right Inches", getEncoderDistanceRightRearT());		// Encoder inches
			// SmartDashboard.putBoolean("Gear High", gearHigh);		//Drivetrain Gear mode
			// SmartDashboard.putBoolean("Brake", brake);					// Brake or Coast
				
			//SmartDashboard.putNumber("Gyro turn angle", getAngle());
			// SmartDashboard.putNumber("Roll", Math708.round(gyro.getRoll(),0));
			// SmartDashboard.putNumber("Pitch", Math708.round(gyro.getPitch(),0));
			SmartDashboard.putNumber("AllianceColor", Robot.allianceColor);
	}
}
