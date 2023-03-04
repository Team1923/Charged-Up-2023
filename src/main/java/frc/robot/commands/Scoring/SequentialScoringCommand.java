// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Scoring;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.StateHandler;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SequentialScoringCommand extends ParallelCommandGroup {
  /** Creates a new SequentialScoringCommand. */
  StateHandler stateHandler = StateHandler.getInstance();
  public SequentialScoringCommand(SwerveSubsystem swerve) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //STEP 1: Start trajectory and hold arm in COBRA
      
        new InstantCommand(() -> stateHandler.setWantToScore(true)),
        new InstantCommand(() -> stateHandler.setHoldInCobra(false)),
        new TrajectoryToGoal(swerve)
      
      //STEP 2: Transition from COBRA to SCORE

      /*
       * Note that ArmDefaultCommand takes care of the rest in terms of 
       * the manipulator. Operator has control over manipulator. Once
       * it is not engaged, the arm will go to COBRA_REVERSE --> STOW
       */
    );
  }
}
