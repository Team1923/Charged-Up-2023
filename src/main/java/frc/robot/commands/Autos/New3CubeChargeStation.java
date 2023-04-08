// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import java.util.HashMap;

import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.SwerveCommands.SwerveXWheels;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.StateHandler;
import frc.robot.util.PathPlannerUtils.AutoFromPathPlanner;
import frc.robot.util.StateVariables.IntakePositions;
import frc.robot.util.StateVariables.IntakeWheelSpeeds;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class New3CubeChargeStation extends SequentialCommandGroup {

  private StateHandler stateHandler = StateHandler.getInstance();
  HashMap<String, Command> eventMap1 = new HashMap<>();
  HashMap<String, Command> eventMap2 = new HashMap<>();
  
  /** Creates a new New3CubeChargeStation. */
  public New3CubeChargeStation(SwerveSubsystem swerve) {

    final AutoFromPathPlanner getToCharge = new AutoFromPathPlanner(swerve, "New3CubeGetToChargeStation", 2.5, 2.5, false, true, true);
    final AutoFromPathPlanner getOnCharge = new AutoFromPathPlanner(swerve, "New3CubeGetOnChargeStation", 0.75, 0.75, false, true, true);
    final AutoFromPathPlanner getOffCharge = new AutoFromPathPlanner(swerve, "New3CubeGetOffChargeStation", 2.5, 2, false, true, true);
    final AutoFromPathPlanner getCubes = new AutoFromPathPlanner(swerve, "New3CubeGetCubes", 3.5, 3, false, true, true);
    final AutoFromPathPlanner mountChargeStation = new AutoFromPathPlanner(swerve, "MountChargeStation", 0.75, 0.75, false, true, true);
    final AutoFromPathPlanner commitBalance = new AutoFromPathPlanner(swerve, "ConfirmBalanceNew3Cube", 1.5, 1.5, false, true, true);


    eventMap1.put("intake_1", new SequentialCommandGroup(
      new InstantCommand(() -> stateHandler.setDesiredIntakePosition(IntakePositions.INTAKE)),
      new InstantCommand(() -> stateHandler.setDesiredIntakeWheelSpeed(IntakeWheelSpeeds.INTAKE))
    ));

    eventMap2.put("auto_shoot_1", new AutoShootSequence());
    eventMap2.put("intake_wheels_1", new InstantCommand(() -> stateHandler.setDesiredIntakeWheelSpeed(IntakeWheelSpeeds.INTAKE)));
    eventMap2.put("bring_intake_up", new SequentialCommandGroup(
      new InstantCommand(() -> stateHandler.setDesiredIntakePosition(IntakePositions.SHOOT_TALL)),
      new InstantCommand(() -> stateHandler.setDesiredIntakeWheelSpeed(IntakeWheelSpeeds.GRIP))
    ));

    addCommands(
      new InstantCommand(() -> swerve.resetModulesToAbsolute()),
      new InstantCommand(() -> swerve.resetOdometryForState(getToCharge.getInitialState())),
      new InstantCommand(() -> stateHandler.setDesiredIntakePosition(IntakePositions.SHOOT_TALL)),
      new InstantCommand(() -> stateHandler.setDesiredIntakeWheelSpeed(IntakeWheelSpeeds.SHOOT_HIGH)),
      new WaitCommand(0.5),
      new InstantCommand(() -> stateHandler.setDesiredIntakeWheelSpeed(IntakeWheelSpeeds.GRIP)),
      getToCharge,
      getOnCharge,
      new InstantCommand(() -> SmartDashboard.putBoolean("GET OFF CHARGE OVER", false)),
      new FollowPathWithEvents(
        getOffCharge,
        getOffCharge.getEventMarkers(),
        eventMap1
      ).withTimeout(2),
      new InstantCommand(() -> SmartDashboard.putBoolean("GET OFF CHARGE OVER", true)),
      new FollowPathWithEvents(
        getCubes,
        getCubes.getEventMarkers(),
        eventMap2
      ),
      mountChargeStation,
      new InstantCommand(() -> stateHandler.setDesiredIntakePosition(IntakePositions.INTAKE_HIGHER)),
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          commitBalance,
          new WaitCommand(0.35),
          new InstantCommand(() -> stateHandler.setDesiredIntakeWheelSpeed(IntakeWheelSpeeds.HIGH_INTAKE_EJECT)),
          new WaitCommand(0.25),  
          new InstantCommand(() -> stateHandler.setDesiredIntakeWheelSpeed(IntakeWheelSpeeds.GRIP)),
          new InstantCommand(() -> stateHandler.setDesiredIntakePosition(IntakePositions.SHOOT_TALL)),
          new InstantCommand(() -> SmartDashboard.putBoolean("ENDED WITH GYRO", false))
        )//,
        // new SequentialCommandGroup(
        //   new WaitUntilCommand(() -> swerve.getAngularVelocity() > 20),//20
        //   new InstantCommand(() -> SmartDashboard.putBoolean("ENDED WITH GYRO", true))
        // )
      ),
      new ParallelCommandGroup(
        new SwerveXWheels(swerve),
        new SequentialCommandGroup(

        )
      ) 


    );
  }
}
