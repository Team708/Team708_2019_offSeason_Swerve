//test jnp 1/12/2019 4:38

package org.usfirst.frc.team708.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.cscore.UsbCamera;

// import org.usfirst.frc.team708.robot.commands.autonomous.*;
import org.usfirst.frc.team708.robot.subsystems.*;
import org.usfirst.frc.team708.robot.Constants;
import org.usfirst.frc.team708.robot.commands.SwerveDrive;



public class Robot extends TimedRobot {

    Timer statsTimer; // Timer used for Smart Dash statistics

    // public static Climber           climber;
    public static Drivetrain        drivetrain;
    // public static Elevator          elevator;
    // public static Intake            intake;
    // public static VisionProcessor   visionProcessor;
    // public static SwerveDrive   swerveDrive;

    public static OI oi;

    public String gameData;
    public String robotLocation;
    public String autoMode;

    public static DriverStation 			ds;
    public static DriverStation.Alliance 	alliance;
    public static int 						allianceColor;

    // public boolean climber = true;
    SendableChooser<Command> autonomousMode = new SendableChooser<>();
    Command autonomousCommand;
    Preferences prefs;

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    public void robotInit() {

        statsTimer = new Timer(); // Initializes the timer for sending Smart Dashboard data
        statsTimer.start(); // Starts the timer for the Smart Dashboard

        // Subsystem Initialization

        // climber         = new Climber();
        //drivetrain      = new Drivetrain();
        // intake          = new Intake();
        // visionProcessor = new VisionProcessor();
        // elevator        = new Elevator();
        drivetrain      = new Drivetrain();


        // visionProcessor.setNTInfo("ledMode", Constants.VISION_LED_OFF);
        // Robot.intake.intakeRetract();    // initialize intake in starting config
        // Robot.intake.hatchRetract();
        // Robot.intake.beakOpen();
        //Robot.drivetrain.shiftGearlow();
        sendDashboardSubsystems(); // Sends each subsystem's cmds to Smart Dashboard

        queueAutonomousModes(); // Adds autonomous modes to the selection box

        // UsbCamera camerafront = CameraServer.getInstance().startAutomaticCapture(0);
        // camerafront.setResolution(160,120);
        // camerafront.setFPS(10);

        // UsbCamera cameraback = CameraServer.getInstance().startAutomaticCapture(1);
        // cameraback.setResolution(160, 120);
        // cameraback.setFPS(15);
        // This MUST BE LAST or a NullPointerException will be thrown
        oi = new OI(); // Initializes the OI
    }

    /**
     * Runs periodically while the robot is enabled
     */
    public void disabledPeriodic() {
        Scheduler.getInstance().run();
        sendStatistics();
        prefs = Preferences.getInstance();

        try {
            // robot is ENABLED
     		if (RobotController.isSysActive()){
                 // connected to FMS
                 ds = DriverStation.getInstance();
                alliance = ds.getAlliance();
	            if (alliance == Alliance.Blue){
                       allianceColor = Constants.ALLIANCE_BLUE;
                }
                else if (alliance == Alliance.Red){
                    allianceColor = Constants.ALLIANCE_RED;
                }
                else {
                    allianceColor = 0;
                }
   
           }
           else // robot is NOT ENABLED
           {allianceColor=20;}
	    
		}
		catch (Exception e)
		{
             allianceColor = 11;
             


    	}
    }

    /**
     * Runs at the start of autonomous mode
     */
    public void autonomousInit() {
        // schedule the autonomous command
        //drivetrain.shiftGearlow();
        //drivetrain.setBrakeMode(true);
        // drivetrain.resetGyro();
        // intake.intakeRetract();
        // intake.beakOpen();
        // intake.hatchRetract();
        // elevator.elev_position = 0.0;
        // original dashboard code
        autonomousCommand = (Command) autonomousMode.getSelected();
        if (autonomousCommand != null)
            autonomousCommand.start();
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
        Scheduler.getInstance().run();
        sendStatistics();
    }

    /**
     * Runs when teleop mode initializes
     */
    public void teleopInit() {
        // This makes sure that the autonomous stops running when teleop starts running.
        // If you want the autonomous to continue until interrupted by another command,
        // remove this line or comment it out.
        if (autonomousCommand != null)
            autonomousCommand.cancel();
        // intake.beakOpen();
        // intake.hatchRetract();
        // intake.intakeRetract();
        //drivetrain.setBrakeMode(false);
        //drivetrain.shiftGearlow();
    }

    /**
     * This function is called when the disabled button is hit. You can use it to
     * reset subsystems before shutting down.
     */
    public void disabledInit() {
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        Scheduler.getInstance().run();
        sendStatistics();
    }

    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
        sendStatistics();
    }

    /**
     * Sends data from each subsystem periodically as set by sendStatsIntervalSec
     */

    private void sendStatistics() {
        // if (statsTimer.get() >= Constants.SEND_STATS_INTERVAL) statsTimer.reset();
        // climber.sendToDashboard();
        //drivetrain.sendToDashboard();
        // elevator.sendToDashboard();
        // intake.sendToDashboard();
        // visionProcessor.sendToDashboard();
    }

    /**
     * Adds every autonomous mode to the selection box and adds the box to the Smart
     * Dashboard
     */
    private void queueAutonomousModes() {
        // autonomousMode.addOption
        // autonomousMode.addOption("Do Nothing", new DoNothing());
        // autonomousMode.addOption("Drive in Square", new DriveInSquare());
        // autonomousMode.addOption("Drive past HAB", new driveDistanceEncoder());
        // autonomousMode.addOption("Rocket Right Near Side", new rocketRightNearSide());
        // autonomousMode.addOption("Rocket Right Far  Side", new rocketRightFarSide());
        // autonomousMode.addOption("Rocket Left  Near Side", new rocketLeftNearSide());
        // autonomousMode.addOption("Cargo  Right Side",  new cargoRightSide());
        // autonomousMode.addOption("Cargo  Left  Side",  new cargoLeftSide());
        // autonomousMode.addOption("Cargo  Right Front", new cargoRightFront());
        // autonomousMode.addOption("Cargo  Left  Front", new cargoLeftFront());

        SmartDashboard.putData("Autonomous Selection", autonomousMode);
    }

    /**
     * Sends every subsystem to the Smart Dashboard
     */
    private void sendDashboardSubsystems() {
        // SmartDashboard.putData(climber);
        //SmartDashboard.putData(drivetrain);
        // SmartDashboard.putData(elevator);
        // SmartDashboard.putData(intake);
        // SmartDashboard.putData(visionProcessor);
    }
}
