// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.util.StateHandler;
import frc.robot.util.StateVariables.IntakePositions;

public class FeederPosition extends CommandBase {

  private StateHandler stateHandler;

  /** Creates a new FeederPosition. */
  public FeederPosition() {
    // Use addRequirements() here to declare subsystem dependencies.
    this.stateHandler = StateHandler.getInstance();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    IntakePositions currentIntakePosition = stateHandler.getCurrentIntakePosition();
    IntakePositions desiredIntakePosition = stateHandler.getDesiredIntakePosition();

    if(currentIntakePosition == IntakePositions.INTAKE && desiredIntakePosition == IntakePositions.INTAKE) {
      stateHandler.setDesiredIntakePosition(IntakePositions.FEED);
      stateHandler.setIntakeInFeed(true);
    } else if(currentIntakePosition == IntakePositions.FEED && desiredIntakePosition == IntakePositions.FEED) {
      stateHandler.setDesiredIntakePosition(IntakePositions.INTAKE);
      stateHandler.setIntakeInFeed(false);
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}