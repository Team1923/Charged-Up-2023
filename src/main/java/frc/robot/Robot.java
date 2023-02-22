// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;

import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.EmergencyCommands.EStopArmCommand;
import frc.robot.commands.EmergencyCommands.EStopIntakeCommand;
import frc.robot.interfaces.AutoChooser;
import frc.robot.interfaces.LEDInterface;
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

  private StateHandler stateHandler = StateHandler.getInstance();
  private LEDInterface ledInterface = LEDInterface.getInstance();

  private AutoChooser selector;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {

    CameraServer.startAutomaticCapture(0);
    PathPlannerServer.startServer(5811);

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.

    robotContainer = new RobotContainer();
    this.selector = new AutoChooser();
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    ledInterface.updateLed();

  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    stateHandler.resetStates();
    stateHandler.setResetManipulator(true);
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {

    stateHandler.setArmGood(Math
        .abs(robotContainer.armSubsystem.getProximalPosition()
            - StateHandler.getInstance().getArmDesiredPosition().getArmAngles().getProximalAngle()) < 0.1
        && Math.abs(
            robotContainer.armSubsystem.getDistalPosition()
                - StateHandler.getInstance().getArmDesiredPosition().getArmAngles().getDistalAngle()) < 0.1);

    stateHandler.setIntakeGood(Math
        .abs(robotContainer.intakeSubsystem.getIntakeProximalPosition()
            - StateHandler.getInstance().getDesiredIntakePosition().getArmAngles().getProximalAngle()) < 0.1
        && Math.abs(
            robotContainer.intakeSubsystem.getIntakeDistalPosition()
                - StateHandler.getInstance().getDesiredIntakePosition().getArmAngles().getDistalAngle()) < 0.3);

    SmartDashboard.putNumber("INTAKE PROXIMAL ERROR", Math
        .abs(robotContainer.intakeSubsystem.getIntakeProximalPosition()
            - StateHandler.getInstance().getDesiredIntakePosition().getArmAngles().getProximalAngle()));

    SmartDashboard.putNumber("INTAKE DISTAL ERROR", Math.abs(
        robotContainer.intakeSubsystem.getIntakeDistalPosition()
            - StateHandler.getInstance().getDesiredIntakePosition().getArmAngles().getDistalAngle()));

    robotContainer.armSubsystem.setCoast();

    SmartDashboard.putBoolean("INTAKE GOOD TO GO", stateHandler.getIsArmGood());

    SmartDashboard.putBoolean("ARM GOOD TO GO", stateHandler.getIsIntakeGood());

  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.initializeAuto(selector);
    robotContainer.armSubsystem.setBrake();

    if (!stateHandler.getIsArmGood()) {
      CommandScheduler.getInstance().schedule(new EStopArmCommand(robotContainer.armSubsystem));
    }

    if (!stateHandler.getIsIntakeGood()) {
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

    if (!stateHandler.getIsArmGood()) {
      CommandScheduler.getInstance().schedule(new EStopArmCommand(robotContainer.armSubsystem));
    }

    if (!stateHandler.getIsIntakeGood()) {
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