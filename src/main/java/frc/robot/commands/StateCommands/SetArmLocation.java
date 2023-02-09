// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.StateCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.util.StateHandler;
import frc.robot.util.StateVariables.VerticalLocations;

public class SetArmLocation extends CommandBase {
  /** Creates a new SetArmLocation. */
  double joystickPOV;
  StateHandler stateHandler = StateHandler.getInstance();

  public SetArmLocation(double joystickPOV) {
    this.joystickPOV = joystickPOV;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (joystickPOV == 0) {
      stateHandler.setVerticalLocation(VerticalLocations.HIGH);
    } else if (joystickPOV == 270) {
      stateHandler.setVerticalLocation(VerticalLocations.MID);
    } else if (joystickPOV == 180) {
      stateHandler.setVerticalLocation(VerticalLocations.LOW);
    } else if (joystickPOV == 90) {
      stateHandler.setVerticalLocation(VerticalLocations.RESET);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
