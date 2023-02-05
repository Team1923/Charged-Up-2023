// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.util.StateHandler;
import frc.robot.util.StateVariables.IntakePositions;

public class SimpleStateIntakeCommand extends CommandBase {
  /** Creates a new SimpleStateIntakeCommand. */
  IntakeSubsystem intake;
  StateHandler stateHandler = StateHandler.getInstance();
  public SimpleStateIntakeCommand(IntakeSubsystem i) {
    this.intake = i;
    addRequirements(intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setIntakeProximalPosition(stateHandler.getDesiredIntakePosition().getArmAngles().getProximalAngle());
    intake.setIntakeDistalPosition(stateHandler.getDesiredIntakePosition().getArmAngles().getDistalAngle());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setIntakeProximalPosition(IntakePositions.STOW.getArmAngles().getProximalAngle());
    intake.setIntakeDistalPosition(IntakePositions.STOW.getArmAngles().getDistalAngle());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
