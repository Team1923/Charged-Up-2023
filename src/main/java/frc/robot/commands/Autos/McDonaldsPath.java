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
public class McDonaldsPath extends SequentialCommandGroup {
  /** Creates a new TestPath. */
  public McDonaldsPath(SwerveSubsystem swerve) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    final AutoFromPathPlanner mcd1 = new AutoFromPathPlanner(swerve, "McDonalds", 2.5, false);
    final AutoFromPathPlanner mcd2 = new AutoFromPathPlanner(swerve, "McDonalds2", 2.5, true);

    addCommands(new InstantCommand(() -> swerve.resetOdometry(mcd1.getInitialPose())), mcd1,
        new InstantCommand(() -> swerve.resetOdometry(mcd2.getInitialPose())), mcd2);
  }
}
