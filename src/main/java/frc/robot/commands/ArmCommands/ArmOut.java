// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmOut extends SequentialCommandGroup {
  /** Creates a new ArmOut. */
  public ArmOut(ArmSubsystem arm) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ArmToPositionExits(arm, ArmConstants.shoulderCobra, ArmConstants.elbowCobra, 0,0, true),
      new ArmToPositionExits(arm, Math.toRadians(50), Math.toRadians(10), 0, 0, true)
    );
  }
}
