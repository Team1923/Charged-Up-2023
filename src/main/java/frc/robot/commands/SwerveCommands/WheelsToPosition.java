// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SwerveCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class WheelsToPosition extends CommandBase {
  /** Creates a new WheelsToPosition. */
  private SwerveSubsystem swerveSubsystem;
  private SwerveModuleState [] states;

  public WheelsToPosition(SwerveSubsystem swerve, double frontLeftAngle, double frontRightAngle,
    double backLeftAngle, double backRightAngle) {
      swerveSubsystem = swerve;
      states = new SwerveModuleState[4];
      states[0] = new SwerveModuleState(1, new Rotation2d(frontLeftAngle));
      states[1] = new SwerveModuleState(1, new Rotation2d(frontRightAngle));
      states[2] = new SwerveModuleState(1, new Rotation2d(backLeftAngle));
      states[3] = new SwerveModuleState(1, new Rotation2d(backRightAngle));
      addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerveSubsystem.setModuleStates(states);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
