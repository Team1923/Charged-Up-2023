// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.SwerveCommands.SwerveXWheels;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.StateHandler;
import frc.robot.util.PathPlannerUtils.AutoFromPathPlanner;
import frc.robot.util.StateVariables.IntakePositions;
import frc.robot.util.StateVariables.IntakeWheelSpeeds;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreCenterBalanceMR extends SequentialCommandGroup {

  private StateHandler stateHandler = StateHandler.getInstance();

  /** Creates a new ScoreCenterBalance. */
  public ScoreCenterBalanceMR(SwerveSubsystem swerve ) {
    
    final AutoFromPathPlanner scoreCenterBalance = new AutoFromPathPlanner(swerve, "ScoreCenterBalanceMR", 3, 3, false, true, true);
    final AutoFromPathPlanner mountStationCommunitySide = new AutoFromPathPlanner(swerve, "MountStationCommunitySide", 0.75, 0.75, false, true, true);
    final AutoFromPathPlanner crossChargeStation = new AutoFromPathPlanner(swerve, "CrossChargeStation", 1.5, 1.5, false, true, true);
    final AutoFromPathPlanner mountChargeStation = new AutoFromPathPlanner(swerve, "MountChargeStation", 0.75, 0.75, false, true, true);


    addCommands(
      new InstantCommand(() -> swerve.resetModulesToAbsolute()),
      new InstantCommand(() -> swerve.resetOdometryForState(scoreCenterBalance.getInitialState())),
      new WaitCommand(1),
      new InstantCommand(() -> stateHandler.setDesiredIntakeWheelSpeed(IntakeWheelSpeeds.SHOOT_HIGH)),
      new WaitCommand(0.5),
      new InstantCommand(() -> stateHandler.setDesiredIntakeWheelSpeed(IntakeWheelSpeeds.GRIP)),
      scoreCenterBalance,
      mountStationCommunitySide,
      crossChargeStation,
      new WaitCommand(0.5),
      mountChargeStation,
      new ConfirmBalanceCommand(swerve),
      new SwerveXWheels(swerve)
    );
  }
}
