package org.usfirst.frc.team708.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.usfirst.frc.team708.robot.Constants;
import org.usfirst.frc.team708.robot.Robot;
import org.usfirst.frc.team708.robot.RobotMap;
import org.usfirst.frc.team708.robot.commands.drivetrain.JoystickDrive;

import edu.wpi.first.wpilibj.command.PIDSubsystem;

public class Drivetrain extends PIDSubsystem {

    
  public static CANSparkMax FL, FR, BL, BR;
  public static TalonSRX steerFL;
  public static TalonSRX steerFR;
  public static TalonSRX steerBL;
  public static TalonSRX steerBR;

  private static final int STATUS_FRAME_PERIOD = 20;

  private CANPIDController driveFLController, driveFRController, driveBLController, driveBRController;

//   private JoystickDrive joystickDrive;

    public Drivetrain() {
        super("Drivetrain", Constants.Kp, Constants.Ki, Constants.Kd);

        FL = new CANSparkMax(RobotMap.DtDriveFrontLeftMotor, MotorType.kBrushless);
        FR = new CANSparkMax(RobotMap.DtDriveFrontRightMotor, MotorType.kBrushless);
        BL = new CANSparkMax(RobotMap.DtDriveBackLeftMotor, MotorType.kBrushless);
        BR = new CANSparkMax(RobotMap.DtDriveBackRightMotor, MotorType.kBrushless);

        FL.setIdleMode(IdleMode.kCoast);
        FR.setIdleMode(IdleMode.kCoast);
        BL.setIdleMode(IdleMode.kCoast);
        BR.setIdleMode(IdleMode.kCoast);
    
        //just in case PID is wanted for the above motors
        driveFLController = new CANPIDController(FL);
        driveFRController = new CANPIDController(FR);
        driveBLController = new CANPIDController(BL);
        driveBRController = new CANPIDController(BR);
    
        //check below with Mr P; we may need separate PID values for the drive and steer; rn they're the same
        //FL
        driveFLController.setP(Constants.Kp);
        driveFLController.setI(Constants.Ki);
        driveFLController.setD(Constants.Kd);
        driveFLController.setSmartMotionMaxAccel(Constants.DRIVE_MOTOR_MAX_ACCEL,0);
    
        //FR
        driveFRController.setP(Constants.Kp);
        driveFRController.setP(Constants.Ki);
        driveFRController.setP(Constants.Kd);
        driveFRController.setSmartMotionMaxAccel(Constants.DRIVE_MOTOR_MAX_ACCEL*Constants.DRIVETRAIN_GEAR_RATIO,0);

        //BL
        driveBLController.setP(Constants.Kp);
        driveBLController.setP(Constants.Kp);
        driveBLController.setP(Constants.Kp);
        driveBLController.setSmartMotionMaxAccel(Constants.DRIVE_MOTOR_MAX_ACCEL,0);

    
        //BR
        driveBRController.setP(Constants.Kp);
        driveBRController.setP(Constants.Kp);
        driveBRController.setP(Constants.Kp);
        driveBRController.setSmartMotionMaxAccel(Constants.DRIVE_MOTOR_MAX_ACCEL,0);

    
    
    
        steerFL = new TalonSRX(RobotMap.DtSteerFrontLeftMotor); // add when given to us
        steerFL.configFactoryDefault();
        steerFL.setInverted(true); //may not be necessary for any of the motors, just check to make sure
        steerFL.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
        
    
        steerFR = new TalonSRX(RobotMap.DtSteerFrontRightMotor); //add when given to us
        steerFR.configFactoryDefault();
        steerFR.setInverted(true);
        steerFR.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);

    
        steerBL = new TalonSRX(RobotMap.DtSteerBackLeftMotor); //add when given to us
        steerBL.configFactoryDefault();
        steerBL.setInverted(true);
        steerBL.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);

        steerBR = new TalonSRX(RobotMap.DtSteerBackRightMotor); //add when given to us
        steerBR.configFactoryDefault();
        steerBR.setInverted(true);
        steerBR.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);

    
        //pid for steer motors - figure out what slotIdx is
        //FL
        steerFL.config_kP(0, Constants.Kp);
        steerFL.config_kI(0, Constants.Ki);
        steerFL.config_kD(0, Constants.Kd);
        steerFL.config_IntegralZone(0, 100, 0); //I don't know what this is but we need it apparently, check with P
        steerFL.configAllowableClosedloopError(0, 5, 0); //same as above
        steerFL.setNeutralMode(NeutralMode.Brake); //check if brake or coast
        steerFL.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, STATUS_FRAME_PERIOD, 0); //check
    
        //FR - repeat same checks as all above lines
        steerFR.config_kP(0, Constants.Kp);
        steerFR.config_kI(0, Constants.Ki);
        steerFR.config_kD(0, Constants.Kd);
        steerFR.config_IntegralZone(0, 100, 0);
        steerFR.configAllowableClosedloopError(0, 5, 0);
        steerFR.setNeutralMode(NeutralMode.Brake);
        steerFR.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, STATUS_FRAME_PERIOD, 0);
    
        //BL - repeat same checks as all above lines
        steerBL.config_kP(0, Constants.Kp);
        steerBL.config_kI(0, Constants.Ki);
        steerBL.config_kD(0, Constants.Kd);
        steerBL.config_IntegralZone(0, 100, 0);
        steerBL.configAllowableClosedloopError(0, 5, 0);
        steerBL.setNeutralMode(NeutralMode.Brake);
        steerBL.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, STATUS_FRAME_PERIOD, 0);
    
        //BR - repeat same checks as all above lines
        steerBR.config_kP(0, Constants.Kp);
        steerBR.config_kI(0, Constants.Ki);
        steerBR.config_kD(0, Constants.Kd);
        steerBR.config_IntegralZone(0, 100, 0);
        steerBR.configAllowableClosedloopError(0, 5, 0);
        steerBR.setNeutralMode(NeutralMode.Brake);
        steerBR.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, STATUS_FRAME_PERIOD, 0);
    
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
        setDefaultCommand(new JoystickDrive());
    }
    

}