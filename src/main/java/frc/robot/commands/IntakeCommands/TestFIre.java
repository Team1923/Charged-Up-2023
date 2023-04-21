// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.util.StateHandler;
import frc.robot.util.StateVariables.IntakeWheelSpeeds;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestFIre extends SequentialCommandGroup {

  StateHandler state = StateHandler.getInstance();

  /** Creates a new TestFIre. */
  public TestFIre() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> state.setDesiredIntakeWheelSpeed(DriverStation.getAlliance() == Alliance.Red ? IntakeWheelSpeeds.FIRST_AUTO_SHOT_RED : IntakeWheelSpeeds.FIRST_AUTO_SHOT_BLUE)),
      new WaitCommand(.5),
      new InstantCommand(() -> state.setDesiredIntakeWheelSpeed(IntakeWheelSpeeds.GRIP))
    );
  }
}
