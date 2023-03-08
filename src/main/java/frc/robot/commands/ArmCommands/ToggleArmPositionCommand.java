// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.util.StateHandler;
import frc.robot.util.StateVariables.ArmPositions;

public class ToggleArmPositionCommand extends CommandBase {
  /** Creates a new ToggleArmPositionCommand. */
  StateHandler stateHandler = StateHandler.getInstance();
  public ToggleArmPositionCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ArmPositions currentArmPosition = stateHandler.getCurrentArmPosition();
    ArmPositions desiredArmPosition = stateHandler.getArmDesiredPosition();


    if(currentArmPosition == ArmPositions.STOW && desiredArmPosition == ArmPositions.STOW) {
      stateHandler.setArmDesiredState(ArmPositions.FEED);
    } else if(currentArmPosition == ArmPositions.FEED && desiredArmPosition == ArmPositions.FEED) {
      stateHandler.setArmDesiredState(ArmPositions.COBRA_REVERSE);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
