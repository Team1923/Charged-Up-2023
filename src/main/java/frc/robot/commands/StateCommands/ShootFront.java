// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.StateCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.util.StateHandler;
import frc.robot.util.StateVariables.IntakePositions;
import frc.robot.util.StateVariables.VerticalLocations;

public class ShootFront extends CommandBase {

  StateHandler stateHandler = StateHandler.getInstance();

  /** Creates a new ShootFront. */
  public ShootFront() {}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stateHandler.setDesiredIntakePosition(     
      IntakePositions.SHOOT_FRONT_HIGH
    );
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
