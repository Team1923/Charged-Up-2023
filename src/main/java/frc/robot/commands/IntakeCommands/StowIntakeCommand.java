// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.util.StateHandler;
import frc.robot.util.StateVariables.IntakePositions;

public class StowIntakeCommand extends CommandBase {
  /** Creates a new StowIntakeCommand. */
  StateHandler stateHandler = StateHandler.getInstance();
  private IntakeSubsystem intake;

  private boolean autoOverride;

  public StowIntakeCommand(IntakeSubsystem intake, boolean autoOverride) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.intake = intake;
    this.autoOverride = autoOverride;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (stateHandler.getCurrentIntakePosition() == IntakePositions.INTAKE) {
      stateHandler.setDesiredIntakePosition(IntakePositions.HANDOFF_1);
      if(autoOverride) {
        stateHandler.setHasGamePiece(true);
      }
      // stateHandler.setHasGamePiece(autoOverride || intake.getGamePieceSensor());
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

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
