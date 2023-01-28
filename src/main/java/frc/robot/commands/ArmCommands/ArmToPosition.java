// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class ArmToPosition extends CommandBase {
  /** Creates a new ElbowToPosition. */
  private ArmSubsystem armSubsystem;
  private double shoulderPosition;
  private double elbowPosition;
  private BooleanSupplier booleanSupplier;
  
  public ArmToPosition(ArmSubsystem a, double sPosition, double ePosition, BooleanSupplier supplier) {
    armSubsystem = a;
    shoulderPosition = sPosition;
    elbowPosition = ePosition;
    booleanSupplier = supplier;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // armSubsystem.setElbowGoal(elbowPosition);
    // armSubsystem.setShoulderGoal(shoulderPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(booleanSupplier.getAsBoolean()){
      armSubsystem.setElbowGoal(elbowPosition);
      armSubsystem.setShoulderGoal(shoulderPosition);
    }
    else{
      armSubsystem.setElbowGoal(ArmConstants.elbowCobra);
      armSubsystem.setShoulderGoal(ArmConstants.shoulderCobra);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("End Command", true);
    armSubsystem.setElbowGoal(ArmConstants.elbowHome);
    armSubsystem.setShoulderGoal(ArmConstants.shoulderHome);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
