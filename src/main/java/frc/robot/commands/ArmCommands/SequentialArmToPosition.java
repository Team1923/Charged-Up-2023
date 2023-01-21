// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SequentialArmToPosition extends SequentialCommandGroup {
  /** Creates a new SequentialArmToPosition. */
  public SequentialArmToPosition(ArmSubsystem armSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ArmToPositionCartesian(armSubsystem, 1, 1),
      new ArmToPositionCartesian(armSubsystem, -1, 1),
      new ArmToPositionCartesian(armSubsystem, -.5, 1),
      new ArmToPositionCartesian(armSubsystem, 0.5, 1),
      new ArmToPositionCartesian(armSubsystem, -0.75, 0.75),
      new ArmToPosition(armSubsystem, ArmConstants.shoulderHome, ArmConstants.elbowHome)

    );
  }
}