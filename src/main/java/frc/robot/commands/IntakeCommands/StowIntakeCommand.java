// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.util.StateHandler;
import frc.robot.util.StateVariables.IntakePositions;

public class StowIntakeCommand extends CommandBase {
  /** Creates a new StowIntakeCommand. */
  StateHandler stateHandler = StateHandler.getInstance();
  public StowIntakeCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    IntakePositions currentIntakePosition = stateHandler.getCurrentIntakePosition();
    switch(currentIntakePosition){
      case STOW:
        stateHandler.setDesiredIntakePosition(IntakePositions.STOW);
        break;
      case INTAKE:
        stateHandler.setDesiredIntakePosition(IntakePositions.HANDOFF_1);
        break;
      case HANDOFF_1:
        stateHandler.setDesiredIntakePosition(IntakePositions.HANDOFF_2);
        break;
      case FINAL_HANDOFF:
        stateHandler.setDesiredIntakePosition(IntakePositions.STOW);
        break;
      default:
        break;
    }
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