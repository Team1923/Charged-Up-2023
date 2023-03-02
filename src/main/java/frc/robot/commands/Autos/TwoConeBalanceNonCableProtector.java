// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import java.util.HashMap;

import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.IntakeCommands.DeployIntakeCommand;
import frc.robot.commands.IntakeCommands.StowIntakeCommand;
import frc.robot.commands.Scoring.ManualScore;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.StateHandler;
import frc.robot.util.PathPlannerUtils.AutoFromPathPlanner;
import frc.robot.util.StateVariables.ArmPositions;
import frc.robot.util.StateVariables.GamePieceMode;
import frc.robot.util.StateVariables.HorizontalLocations;
import frc.robot.util.StateVariables.IntakePositions;
import frc.robot.util.StateVariables.VerticalLocations;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoConeBalanceNonCableProtector extends SequentialCommandGroup {
  /** Creates a new TwoConeLeft. */
  private StateHandler stateHandler = StateHandler.getInstance();

  public TwoConeBalanceNonCableProtector(SwerveSubsystem swerve, IntakeSubsystem intake) {

    final AutoFromPathPlanner acquireCone = new AutoFromPathPlanner(swerve, "AcquireConeNOCP", 2.5, 2, false, true,
        true);
    final AutoFromPathPlanner scoreCone = new AutoFromPathPlanner(swerve, "ScoreConeNOCP", 2.5, 2, false, true, true);
    final AutoFromPathPlanner balance = new AutoFromPathPlanner(swerve, "BalanceNOCP", 2.5, 2, false, true, true);

    addCommands(
        new InstantCommand(() -> swerve.resetOdometry(acquireCone.getInitialPose())),
        new InstantCommand(() -> swerve.zeroGyro(acquireCone.getInitialPose().getRotation().getDegrees())),

        /*
         * The first line sets the desiredArmPosition.
         * When the arm is in position, the manipulator disengages.
         * When this is done, we then go back to STOW
         */
        new ParallelCommandGroup(
            new SequentialCommandGroup(
                new AutoScoreCommand(HorizontalLocations.RIGHT, VerticalLocations.HIGH, GamePieceMode.CONE),
                acquireCone
            ),
            new SequentialCommandGroup(
                new InstantCommand(() -> SmartDashboard.putNumber("DEBUG", 0)),
                new WaitUntilCommand(() -> stateHandler.getResetManipulator()),
                new InstantCommand(() -> SmartDashboard.putNumber("DEBUG", 1)),
                new InstantCommand(() -> stateHandler.setDesiredIntakePosition(IntakePositions.REVERSE_HANDOFF_1)),
                new InstantCommand(() -> SmartDashboard.putNumber("DEBUG", 2)),
                new InstantCommand(() -> stateHandler.setAutoRunIntake(true)), // set intake wheel speeds
                new InstantCommand(() -> SmartDashboard.putNumber("DEBUG", 3))
            )),
        // new ParallelCommandGroup(
        //     /*
        //      * The sequential command group below will first
        //      * wait for the arm to be in stow,
        //      * then the intake will deploy
        //      */
        //     // new SequentialCommandGroup(
        //         //new WaitUntilCommand(() -> stateHandler.getCurrentArmPosition() == ArmPositions.COBRA_REVERSE),
        //     ),
        // assuming cone has been intaked, no need to run the wheels anymore
        new InstantCommand(() -> stateHandler.setAutoRunIntake(false)),


        new ParallelCommandGroup(
            new SequentialCommandGroup(
                new InstantCommand(() -> SmartDashboard.putNumber("I DEBUG", 0)),
                new StowIntakeCommand(intake, () -> true),
                new InstantCommand(() -> SmartDashboard.putNumber("I DEBUG", 1)),
                new WaitUntilCommand(() -> stateHandler.getCurrentIntakePosition() == IntakePositions.FINAL_HANDOFF),
                new InstantCommand(() -> SmartDashboard.putNumber("I DEBUG", 2)),
                new AutoArmToPosition(HorizontalLocations.LEFT, VerticalLocations.HIGH, GamePieceMode.CONE),
                new InstantCommand(() -> SmartDashboard.putNumber("I DEBUG", 3))
            ),
            scoreCone
            ),


        new InstantCommand(() -> stateHandler.setResetManipulator(true))

     
        // balance

    );
  }
}
