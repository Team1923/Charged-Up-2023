// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmDefaultCommand extends CommandBase {
  /** Creates a new ArmDefaultCommand. */
  private ArmSubsystem armSubsystem;
  private double shoulderGoal;
  private double elbowGoal;
  public ArmDefaultCommand(ArmSubsystem aSubsystem) {
    armSubsystem = aSubsystem;
    shoulderGoal = 0;
    elbowGoal = 0;
    addRequirements(armSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shoulderGoal = armSubsystem.getShoulderGoal();
    elbowGoal = armSubsystem.getElbowGoal();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shoulderGoal = armSubsystem.getShoulderGoal();
    elbowGoal = armSubsystem.getElbowGoal();

    armSubsystem.setShoulderVoltage(armSubsystem.getShoulderPIDOutput() + armSubsystem.calculateShoulderFeedforward());
    armSubsystem.setElbowVoltage(armSubsystem.getElbowPIDOutput() + armSubsystem.calculateElbowFeedforward());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
