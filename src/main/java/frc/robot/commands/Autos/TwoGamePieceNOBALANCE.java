// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.IntakeCommands.StowIntakeCommand;
import frc.robot.commands.SwerveCommands.SwerveXWheels;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.StateHandler;
import frc.robot.util.PathPlannerUtils.AutoFromPathPlanner;
import frc.robot.util.StateVariables.GamePieceMode;
import frc.robot.util.StateVariables.HorizontalLocations;
import frc.robot.util.StateVariables.IntakePositions;
import frc.robot.util.StateVariables.VerticalLocations;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoGamePieceNOBALANCE extends SequentialCommandGroup {
  /** Creates a new TwoConeLeft. */
  private StateHandler stateHandler = StateHandler.getInstance();

  public TwoGamePieceNOBALANCE(SwerveSubsystem swerve, IntakeSubsystem intake) {

    final AutoFromPathPlanner acquireCube = new AutoFromPathPlanner(swerve, "AcquireFirstCube", 3.5, 3, false, true,
        true);
    final AutoFromPathPlanner scoreCube = new AutoFromPathPlanner(swerve, "ScoreCube", 3.5, 3, false, true, true);

    addCommands(

        new InstantCommand(() -> swerve.resetOdometryForState(acquireCube.getInitialState())),
        new InstantCommand(() -> stateHandler.setWantToUpdateOdometry(false)),
        new InstantCommand(() -> stateHandler.setDesiredIntakePosition(IntakePositions.EJECT)),
        new WaitUntilCommand(() -> stateHandler.getCurrentIntakePosition() == IntakePositions.EJECT),
        new WaitCommand(0.75),
        new InstantCommand(() -> stateHandler.setVerticalLocation(VerticalLocations.HIGH)),
        new InstantCommand(() -> stateHandler.setAutoEjecting(true)),
        new WaitCommand(1),
        new InstantCommand(() -> stateHandler.setAutoEjecting(false)),
        new ParallelCommandGroup(
            acquireCube,
            new SequentialCommandGroup(
                new WaitCommand(1.5),
                new InstantCommand(() -> stateHandler.setDesiredIntakePosition(IntakePositions.INTAKE)),
                new InstantCommand(() -> stateHandler.setAutoRunIntake(true))

            )
        ),
        new ParallelCommandGroup(
            scoreCube,
            new SequentialCommandGroup(
                new WaitCommand(0.25),
                new InstantCommand(() -> stateHandler.setDesiredIntakePosition(IntakePositions.EJECT)),
                new InstantCommand(() -> stateHandler.setAutoRunIntake(false))

            )
        ),
        new WaitUntilCommand(() -> stateHandler.getCurrentIntakePosition() == IntakePositions.EJECT),
        new WaitCommand(0.75),
        new InstantCommand(() -> stateHandler.setVerticalLocation(VerticalLocations.LOW)),
        new InstantCommand(() -> stateHandler.setAutoEjecting(true)),
        new WaitCommand(1),
        new InstantCommand(() -> stateHandler.setAutoEjecting(false)),
        new InstantCommand(() -> stateHandler.setDesiredIntakePosition(IntakePositions.STOW))


        // new InstantCommand(() -> swerve.zeroGyro(acquireCone.getInitialPose().getRotation().getDegrees())),

        // // new InstantCommand(() -> stateHandler.setArmDesiredState(ArmPositions.COBRA_FORWARD)),
    
        // new ParallelCommandGroup(
        //     new SequentialCommandGroup(
        //         new AutoScoreCommand(HorizontalLocations.RIGHT, VerticalLocations.HIGH, GamePieceMode.CONE),
        //         acquireCone
        //     ),

        //     new SequentialCommandGroup(
        //         new WaitUntilCommand(() -> stateHandler.getResetManipulator()),
        //         new InstantCommand(() -> stateHandler.setResetManipulator(false)),
        //         new InstantCommand(() -> stateHandler.setDesiredIntakePosition(IntakePositions.REVERSE_HANDOFF_1)),
        //         new InstantCommand(() -> stateHandler.setAutoRunIntake(true)) // set intake wheel speeds
        //     )
        // ),
       
        // new InstantCommand(() -> stateHandler.setAutoRunIntake(false)),


        // new ParallelCommandGroup(
        //     new SequentialCommandGroup(
        //         new StowIntakeCommand(intake, true),
        //         new WaitUntilCommand(() -> stateHandler.getCurrentIntakePosition() == IntakePositions.FINAL_HANDOFF),
        //         new AutoArmToPosition(HorizontalLocations.LEFT, VerticalLocations.HIGH, GamePieceMode.CONE)
        //     ),
        //     scoreCone
        // ),


        // new InstantCommand(() -> stateHandler.setResetManipulator(true)),
        // new WaitCommand(0.1),
        // new InstantCommand(() -> stateHandler.setResetManipulator(false))
    );
  }
}