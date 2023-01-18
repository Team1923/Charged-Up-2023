// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.reflect.Array;
import java.util.Arrays;
import java.util.ResourceBundle.Control;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.interfaces.LimelightInterface;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;

  // private LimelightInterface limelight = LimelightInterface.getInstance();

  // private final double txtune = 0.01;
  // private final double tdtune = 0.005;

  // private WPI_TalonFX leftdriveprimary = new WPI_TalonFX(1);
  // private WPI_TalonFX leftdrive1 = new WPI_TalonFX(2);
  // private WPI_TalonFX leftdrive2 = new WPI_TalonFX(3);
  // private WPI_TalonFX rightdriveprimary = new WPI_TalonFX(4);
  // private WPI_TalonFX rightdrive1 = new WPI_TalonFX(5);
  // private WPI_TalonFX rightdrive2 = new WPI_TalonFX(6);



  // public double RotatetoTag(){
  //   double error = limelight.getHorizontalOffset();
  //  if(Math.abs(error) > 2){
  //    return  - limelight.getHorizontalOffset()*txtune;
  //  }
  //  else{
  //   return 0;
  //  }
  // }

  // public double GettoTag(){
  //   double error = limelight.distanceToTarget();
  //   if(error > 20){
  //     return limelight.distanceToTarget()*tdtune;
  //   }
  //   else{
  //     return  0;
  //   }
  // }

  // public void Drive() {

  //   double rot = RotatetoTag();
  //   double trans = GettoTag();

  //   SmartDashboard.putNumber("Rotational", rot);
  //   SmartDashboard.putNumber("Translational", trans);
  //   SmartDashboard.putNumber("Distance to Target", limelight.distanceToTarget());

  //   leftdriveprimary.set(ControlMode.PercentOutput, rot + trans);
  //   rightdriveprimary.set(ControlMode.PercentOutput, rot - trans);

  // }
  


  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {


    Logger.getInstance().recordMetadata("ProjectName", "MyProject"); // Set a metadata value

if (true) {//isReal()) {
    Logger.getInstance().addDataReceiver(new WPILOGWriter("/media/sda1/")); // Log to a USB stick
    Logger.getInstance().addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
} else {
    setUseTiming(false); // Run as fast as possible
    String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
    Logger.getInstance().setReplaySource(new WPILOGReader(logPath)); // Read replay log
    Logger.getInstance().addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
}

    Logger.getInstance().start();


    Logger logger = Logger.getInstance();

    // Record metadata
    logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      // Running on a real robot, log to a USB stick
      case REAL:
        logger.addDataReceiver(new WPILOGWriter("/media/sda1/"));
        logger.addDataReceiver(new NT4Publisher());
        break;

      // Running a physics simulator, log to local folder
      case SIM:
        logger.addDataReceiver(new WPILOGWriter(""));
        logger.addDataReceiver(new NT4Publisher());
        break;

      // Replaying a log, set up replay source
      case REPLAY:
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        logger.setReplaySource(new WPILOGReader(logPath));
        logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Start AdvantageKit logger
    logger.start();
    
    // leftdrive1.configFactoryDefault();
    // leftdrive2.configFactoryDefault();
    // rightdrive1.configFactoryDefault();
    // rightdrive2.configFactoryDefault();
    // leftdriveprimary.configFactoryDefault();
    // rightdriveprimary.configFactoryDefault();


    // leftdrive1.follow(leftdriveprimary);
    // leftdrive2.follow(leftdriveprimary);
    // rightdrive1.follow(rightdriveprimary);
    // rightdrive2.follow(rightdriveprimary);

    // leftdriveprimary.setInverted(InvertType.None);
    // leftdrive1.setInverted(InvertType.FollowMaster);
    // leftdrive2.setInverted(InvertType.FollowMaster);


    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.
    CommandScheduler.getInstance().run();

    LimelightInterface limelight = LimelightInterface.getInstance();

  

    SmartDashboard.putNumber("Horizontal Offset", limelight.getHorizontalOffset());
    SmartDashboard.putNumber("Verticle Offset", limelight.getVerticalOffset());
    SmartDashboard.putNumber("Area", limelight.getArea());
    SmartDashboard.putNumber("Skew", limelight.getSkew());
    SmartDashboard.putNumberArray("Camera Transformation", limelight.getCamtransformation());
    SmartDashboard.putNumber("Tag ID", limelight.getID());
    SmartDashboard.putString("Botpose", Arrays.toString(limelight.getBotPose()));
   
    


    
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //Drive();
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
