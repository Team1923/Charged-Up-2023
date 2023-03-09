// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SwerveCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveXWheels extends CommandBase {

  private SwerveSubsystem swerve;

  private SwerveModuleState[] xStates = {
    new SwerveModuleState(0.1, Rotation2d.fromDegrees(-135)),
    new SwerveModuleState(0.1, Rotation2d.fromDegrees(135)),
    new SwerveModuleState(0.1, Rotation2d.fromDegrees(-45)),
    new SwerveModuleState(0.1, Rotation2d.fromDegrees(45)),
  };

  /** Creates a new SwerveXWheels. */
  public SwerveXWheels(SwerveSubsystem s) {
    this.swerve = s;
    addRequirements(swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerve.updateModuleStates(xStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.

  @Override
  public boolean isFinished() {
    return false;
  }
}
