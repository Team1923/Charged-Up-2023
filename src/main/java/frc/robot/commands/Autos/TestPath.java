// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.PathPlannerUtils.AutoFromPathPlanner;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestPath extends SequentialCommandGroup {
  /** Creates a new TestPath. */
  public TestPath(SwerveSubsystem swerve) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    final AutoFromPathPlanner testPath = new AutoFromPathPlanner(swerve, "TestPath", 2.5, true);

    addCommands(new InstantCommand(() -> swerve.resetOdometry(testPath.getInitialPose())),
        new InstantCommand(() -> swerve.zeroGyro(testPath.getInitialPose().getRotation().getDegrees())), testPath);
  }
}
