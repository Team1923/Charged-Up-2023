// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import java.util.HashMap;

import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.PathPlannerUtils.AutoFromPathPlanner;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestMarkers extends SequentialCommandGroup {
  /** Creates a new TestPath. */
  HashMap<String, Command> eventMap = new HashMap<>();
  
  public TestMarkers(SwerveSubsystem swerve) {

    final AutoFromPathPlanner testPath = new AutoFromPathPlanner(swerve, "New Path", 2.5, 2, false, false, true);
    eventMap.put("midway", new PrintCommand("MIDWAY THROUGH PATH"));



    addCommands(
      new InstantCommand(() -> swerve.resetOdometryForState(testPath.getInitialState())),
      new FollowPathWithEvents(
        testPath,
        testPath.getEventMarkers(),
        eventMap
      )
    );
  }
}
