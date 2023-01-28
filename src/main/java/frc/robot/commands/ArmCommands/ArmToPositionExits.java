// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class ArmToPositionExits extends CommandBase {
  /** Creates a new ElbowToPosition. */
  private ArmSubsystem armSubsystem;

  private double shoulderPosition;
  private double elbowPosition;

  private double endShoulderPosition;
  private double endElbowPosition;

  private boolean holdOnEnd;

  private double angleThreshold = Math.toRadians(1.5);


  private double timeThreshold = 5;
  private int counter;

  public ArmToPositionExits(ArmSubsystem a, double startShoulder, 
  double startElbow, double endShoulder, double endElbow, boolean holdOnEnd) {
    armSubsystem = a;
    this.shoulderPosition = startShoulder;
    this.elbowPosition = startElbow;
    this.endShoulderPosition = endShoulder;
    this.endElbowPosition = endElbow;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    counter = 0;
    armSubsystem.setElbowGoal(elbowPosition);
    armSubsystem.setShoulderGoal(shoulderPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("End Command", true);
    if(!holdOnEnd) {
      armSubsystem.setElbowGoal(endElbowPosition);
      armSubsystem.setShoulderGoal(endShoulderPosition);
    }
  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double elbowError = Math.abs(armSubsystem.getElbowGoal() - armSubsystem.getElbowPosition());
    double shoulderError = Math.abs(armSubsystem.getShoulderGoal() - armSubsystem.getShoulderPosition());

    if(shoulderError < angleThreshold && elbowError < angleThreshold) {
      counter++;
    } else {
      counter = 0;
    }

    return counter > timeThreshold;
  }
}
