// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.SwerveCommands.SwerveXWheels;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.StateHandler;
import frc.robot.util.PathPlannerUtils.AutoFromPathPlanner;
import frc.robot.util.StateVariables.GamePieceMode;
import frc.robot.util.StateVariables.HorizontalLocations;
import frc.robot.util.StateVariables.VerticalLocations;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreCenterAndBalance extends SequentialCommandGroup {
  /** Creates a new ScoreCenterAndBalance. */

  private StateHandler stateHandler = StateHandler.getInstance();

  public ScoreCenterAndBalance(SwerveSubsystem swerve) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    final AutoFromPathPlanner balance = new AutoFromPathPlanner(swerve, "ScoreCenterBalance", 2.5, 2, false, true, true);

    addCommands(
      new InstantCommand(() -> swerve.resetOdometry(balance.getInitialPose())),
      new InstantCommand(() -> swerve.zeroGyro(balance.getInitialPose().getRotation().getDegrees())),

      new AutoScoreCommand(HorizontalLocations.LEFT, VerticalLocations.HIGH, GamePieceMode.CONE),
      new WaitUntilCommand(() -> stateHandler.getResetManipulator()),
      new InstantCommand(() -> stateHandler.setResetManipulator(false)),
      new ParallelRaceGroup(
            // GYRO VELOCITY MEASUREMENTS
            new WaitUntilCommand(() -> swerve.getAngularVelocity() < -15),
            balance
      ),
      new SwerveXWheels(swerve)

    );
  }
}
