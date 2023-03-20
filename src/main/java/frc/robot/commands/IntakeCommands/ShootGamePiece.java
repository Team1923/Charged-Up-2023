// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.util.StateHandler;
import frc.robot.util.StateVariables.IntakeWheelSpeeds;
import frc.robot.util.StateVariables.VerticalLocations;

public class ShootGamePiece extends CommandBase {
  /** Creates a new ShootGamePiece. */

  StateHandler stateHandler = StateHandler.getInstance();

  IntakeWheelSpeeds desiredShootSpeed = IntakeWheelSpeeds.GRIP;

  public ShootGamePiece() {
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(stateHandler.getCurrentVerticalLocation() == VerticalLocations.LOW) {
      desiredShootSpeed = IntakeWheelSpeeds.SHOOT_LOW;
    } else if(stateHandler.getCurrentVerticalLocation() == VerticalLocations.MID) {
      desiredShootSpeed = IntakeWheelSpeeds.SHOOT_MID;
    } else {
      desiredShootSpeed = IntakeWheelSpeeds.SHOOT_HIGH;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    stateHandler.setDesiredIntakeWheelSpeed(desiredShootSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    stateHandler.setDesiredIntakeWheelSpeed(IntakeWheelSpeeds.GRIP);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
