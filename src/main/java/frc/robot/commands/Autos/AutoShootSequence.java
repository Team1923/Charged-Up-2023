// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.util.StateHandler;
import frc.robot.util.StateVariables.IntakePositions;
import frc.robot.util.StateVariables.IntakeWheelSpeeds;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoShootSequence extends SequentialCommandGroup {

  private StateHandler stateHandler = StateHandler.getInstance();

  /** Creates a new AutoShootSequence. */
  public AutoShootSequence() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(    
      new InstantCommand(() -> stateHandler.setDesiredIntakeWheelSpeed(IntakeWheelSpeeds.INTAKE)),
      new InstantCommand(() -> stateHandler.setDesiredIntakePosition(IntakePositions.SHOOT_SMALL)),
      new WaitCommand(0.75),
      new InstantCommand(() -> stateHandler.setStickOut(true)),
      new WaitCommand(0.6),
      new InstantCommand(() -> stateHandler.setDesiredIntakeWheelSpeed(IntakeWheelSpeeds.CHARGE_STATION_PLOP)),
      new InstantCommand(() -> stateHandler.setStickOut(false)),
      new WaitCommand(0.25),
      new InstantCommand(() -> stateHandler.setDesiredIntakeWheelSpeed(IntakeWheelSpeeds.GRIP)),
      new InstantCommand(() -> stateHandler.setDesiredIntakePosition(IntakePositions.INTAKE))
    );
  }

  public AutoShootSequence(boolean test) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(    
      new InstantCommand(() -> stateHandler.setDesiredIntakeWheelSpeed(IntakeWheelSpeeds.INTAKE)),
      new InstantCommand(() -> stateHandler.setDesiredIntakePosition(IntakePositions.SHOOT_SMALL)),
      new WaitCommand(0.75),
      // new InstantCommand(() -> stateHandler.setDesiredIntakeWheelSpeed(IntakeWheelSpeeds.GRIP)),
      new InstantCommand(() -> stateHandler.setStickOut(true)),
      new WaitCommand(0.5),//0.6
      new InstantCommand(() -> stateHandler.setDesiredIntakeWheelSpeed(IntakeWheelSpeeds.CHARGE_STATION_PLOP)),
      new InstantCommand(() -> stateHandler.setStickOut(false)),
      new WaitCommand(0.15),
      new InstantCommand(() -> stateHandler.setDesiredIntakeWheelSpeed(IntakeWheelSpeeds.GRIP)),
      new InstantCommand(() -> stateHandler.setDesiredIntakePosition(IntakePositions.INTAKE))
    );
  }
}
