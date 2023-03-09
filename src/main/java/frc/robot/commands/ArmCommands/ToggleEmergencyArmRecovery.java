// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ToggleEmergencyArmRecovery extends CommandBase {

  private ArmSubsystem arm;
  private DoubleSupplier proximal, distal;

  /** Creates a new ToggleEmergencyArmRecovery. */
  public ToggleEmergencyArmRecovery(ArmSubsystem a, DoubleSupplier prox, DoubleSupplier dist) {
    this.arm = a;
    this.proximal = prox;
    this.distal = dist;
    addRequirements(arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.setMotorOutputs(proximal.getAsDouble() * 0.1, distal.getAsDouble() * 0.1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("IN COMMAND?", false);
    arm.overrideMotorZeros();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
