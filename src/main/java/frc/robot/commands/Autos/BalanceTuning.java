// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import java.util.HashMap;

import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.StateHandler;
import frc.robot.util.PathPlannerUtils.AutoFromPathPlanner;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BalanceTuning extends SequentialCommandGroup {
  /** Creates a new BalanceTuning. */
  public BalanceTuning(SwerveSubsystem swerve) {

    final AutoFromPathPlanner FiveCubeAuto = new AutoFromPathPlanner(swerve, "5CubeAuto", 3, 3, false, true, true);
    StateHandler stateHandler = StateHandler.getInstance();
    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("measure_gyro", new InstantCommand(() -> stateHandler.setUseGyroVelocityMeasurement(true)));

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ParallelRaceGroup(
            new FollowPathWithEvents(FiveCubeAuto,
                FiveCubeAuto.getEventMarkers(),
                eventMap),

            new SequentialCommandGroup(
                new WaitUntilCommand(() -> stateHandler.getUseGyroVelocityMeasurement()),
                new ConfirmBalanceCommand(swerve))

        ));
  }
}
