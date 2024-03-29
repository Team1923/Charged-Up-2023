// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import java.util.HashMap;

import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.SwerveCommands.SwerveXWheels;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.StateHandler;
import frc.robot.util.PathPlannerUtils.AutoFromPathPlanner;
import frc.robot.util.StateVariables.IntakePositions;
import frc.robot.util.StateVariables.IntakeWheelSpeeds;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FiveCubeNoBalanceMallRat extends SequentialCommandGroup {

  private StateHandler stateHandler = StateHandler.getInstance();
  HashMap<String, Command> eventMap = new HashMap<>();

  /** Creates a new FourCubeWithBalance. */
  public FiveCubeNoBalanceMallRat(SwerveSubsystem swerve) {

    final AutoFromPathPlanner fiveNoBalance = new AutoFromPathPlanner(swerve, "5CubeAutoMallRat", 1.5, 1.5, false, true, true);
    eventMap.put("shoot_1", new AutoShootSequence(true));
    eventMap.put("shoot_2", new AutoShootSequence());
    eventMap.put("shoot_3", new AutoShootSequence());
    eventMap.put("shoot_4", new AutoShootSequence());


    eventMap.put("intake_1",
        new InstantCommand(() -> stateHandler.setDesiredIntakeWheelSpeed(IntakeWheelSpeeds.INTAKE)));
    eventMap.put("intake_2",
        new InstantCommand(() -> stateHandler.setDesiredIntakeWheelSpeed(IntakeWheelSpeeds.INTAKE)));
    eventMap.put("intake_3",
        new InstantCommand(() -> stateHandler.setDesiredIntakeWheelSpeed(IntakeWheelSpeeds.INTAKE)));
    eventMap.put("intake_4",
        new InstantCommand(() -> stateHandler.setDesiredIntakeWheelSpeed(IntakeWheelSpeeds.INTAKE)));

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new InstantCommand(() -> swerve.resetModulesToAbsolute()),
        new InstantCommand(() -> swerve.resetOdometryForState(fiveNoBalance.getInitialState())),
        new ParallelCommandGroup(
            new ParallelCommandGroup(
                new WaitCommand(0.25),
                new SequentialCommandGroup(
                    new FollowPathWithEvents(
                        fiveNoBalance,
                        fiveNoBalance.getEventMarkers(),
                        eventMap),
                    new InstantCommand(() -> stateHandler.setDesiredIntakePosition(IntakePositions.SHOOT_TALL))
                )
            ),
            new SequentialCommandGroup(
                new InstantCommand(() -> stateHandler.setDesiredIntakeWheelSpeed(IntakeWheelSpeeds.CHARGE_STATION_PLOP)),
                new WaitCommand(0.25),
                new InstantCommand(() -> stateHandler.setDesiredIntakeWheelSpeed(IntakeWheelSpeeds.GRIP)),
                new InstantCommand(() -> stateHandler.setDesiredIntakePosition(IntakePositions.INTAKE))
            )
        )
    );
  }



}
