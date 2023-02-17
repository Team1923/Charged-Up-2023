// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.EmergencyCommands.EStopArmCommand;
import frc.robot.commands.EmergencyCommands.EStopIntakeCommand;
import frc.robot.util.StateHandler;

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

  public static CTREConfigs ctreConfigs = new CTREConfigs();

  private boolean armGood = false;
  private boolean intakeGood = false;


  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {

    CameraServer.startAutomaticCapture(0);

    Logger.getInstance().recordMetadata("ProjectName", "MyProject"); // Set a metadata value

    if (true) {// isReal()) {
      Logger.getInstance().addDataReceiver(new WPILOGWriter("/media/sda1/")); // Log to a USB stick
      Logger.getInstance().addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
      new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
    } else {
      setUseTiming(false); // Run as fast as possible
      String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
      Logger.getInstance().setReplaySource(new WPILOGReader(logPath)); // Read replay log
      Logger.getInstance().addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save
                                                                                                          // outputs to
                                                                                                          // a new log
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
    // logger.start();

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.

    robotContainer = new RobotContainer();
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    StateHandler.getInstance().resetStates();
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {

    armGood = Math
    .abs(robotContainer.armSubsystem.getProximalPosition()
        - StateHandler.getInstance().getArmDesiredPosition().getArmAngles().getProximalAngle()) < 0.1
    && Math.abs(
        robotContainer.armSubsystem.getDistalPosition()
            - StateHandler.getInstance().getArmDesiredPosition().getArmAngles().getDistalAngle()) < 0.1;

    intakeGood = Math
    .abs(robotContainer.intakeSubsystem.getIntakeProximalPosition()
        - StateHandler.getInstance().getDesiredIntakePosition().getArmAngles().getProximalAngle()) < 0.1
    && Math.abs(
        robotContainer.intakeSubsystem.getIntakeDistalPosition()
            - StateHandler.getInstance().getDesiredIntakePosition().getArmAngles().getDistalAngle()) < 0.3;

    SmartDashboard.putNumber("INTAKE PROXIMAL ERROR", Math
    .abs(robotContainer.intakeSubsystem.getIntakeProximalPosition()
        - StateHandler.getInstance().getDesiredIntakePosition().getArmAngles().getProximalAngle()));
    
    SmartDashboard.putNumber("INTAKE DISTAL ERROR", Math.abs(
      robotContainer.intakeSubsystem.getIntakeDistalPosition()
          - StateHandler.getInstance().getDesiredIntakePosition().getArmAngles().getDistalAngle()));

    robotContainer.armSubsystem.setCoast();

    SmartDashboard.putBoolean("INTAKE GOOD TO GO", intakeGood);

    SmartDashboard.putBoolean("ARM GOOD TO GO", armGood);
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    if(!armGood) {
      CommandScheduler.getInstance().schedule(new EStopArmCommand(robotContainer.armSubsystem));
    }

    if(!intakeGood) {
      CommandScheduler.getInstance().schedule(new EStopIntakeCommand(robotContainer.intakeSubsystem));
    }

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

    if(!armGood) {
      CommandScheduler.getInstance().schedule(new EStopArmCommand(robotContainer.armSubsystem));
    }

    if(!intakeGood) {
      CommandScheduler.getInstance().schedule(new EStopIntakeCommand(robotContainer.intakeSubsystem));
    }

    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
    robotContainer.armSubsystem.setBrake();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // Drive();

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
