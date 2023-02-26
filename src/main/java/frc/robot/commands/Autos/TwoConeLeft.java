// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.IntakeCommands.DeployIntakeCommand;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.StateHandler;
import frc.robot.util.PathPlannerUtils.AutoFromPathPlanner;
import frc.robot.util.StateVariables.ArmPositions;
import frc.robot.util.StateVariables.GamePieceMode;
import frc.robot.util.StateVariables.HorizontalLocations;
import frc.robot.util.StateVariables.VerticalLocations;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoConeLeft extends SequentialCommandGroup {
  /** Creates a new TwoConeLeft. */
  private StateHandler stateHandler = StateHandler.getInstance();

  public TwoConeLeft(SwerveSubsystem swerve) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    final AutoFromPathPlanner acquireCone = new AutoFromPathPlanner(swerve, "AcquireCone", 2.5, true);

    addCommands(
      new InstantCommand(() -> swerve.resetOdometry(acquireCone.getInitialPose())),
      new InstantCommand(() -> swerve.zeroGyro(acquireCone.getInitialPose().getRotation().getDegrees())),

      /*
       * The first line sets the desiredArmPosition.
       * When the arm is in position, the manipulator disengages.
       * When this is done, we then go back to STOW
       */
      new AutoScoreCommand(HorizontalLocations.LEFT, VerticalLocations.HIGH, GamePieceMode.CONE),
      new ParallelCommandGroup(
        new InstantCommand(() -> stateHandler.setAutoRunIntake(true)),
        /* The sequential command group below will first 
         * wait for the arm to be in stow
         */
        new SequentialCommandGroup(
          new WaitUntilCommand(() -> stateHandler.getCurrentArmPosition() == ArmPositions.STOW),
          new DeployIntakeCommand()
        ),
        acquireCone
      ),
      new InstantCommand(() -> stateHandler.setAutoRunIntake(false))
      
    );
  }
}
