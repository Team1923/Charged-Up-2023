// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.util.StateHandler;
import frc.robot.util.StateVariables.ArmPositions;
import frc.robot.util.StateVariables.GamePieceMode;
import frc.robot.util.StateVariables.IntakePositions;

public class IntakeArmDefaultCommand extends CommandBase {

  private IntakeSubsystem intake;
  private StateHandler stateHandler;
  private BooleanSupplier intakeSupplier;
  private BooleanSupplier ejectSupplier;

  /** Creates a new IntakeArmDefaultCommand. */
  public IntakeArmDefaultCommand(IntakeSubsystem i, BooleanSupplier intakeSupplier, BooleanSupplier ejectSupplier) {
    intake = i;
    this.intakeSupplier = intakeSupplier;
    this.ejectSupplier = ejectSupplier;
    addRequirements(intake);

    this.stateHandler = StateHandler.getInstance();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    IntakePositions currentDesiredState = stateHandler.getDesiredIntakePosition();

    switch (currentDesiredState) {
      case INTAKE_CUBE:
        break;
      case HOLD:
        break;
      case CUBE_HANDOFF:
        break;
      case STOW:
        break;
      case INTAKE_CONE:
        break;
      case CONE_HANDOFF:
        break;
      case RETRACT:
        break;
      default:
        break;
    }

    currentDesiredState = stateHandler.getDesiredIntakePosition();

    intake.setIntakeProximalPosition(currentDesiredState.getArmAngles().getProximalAngle());
    intake.setIntakeDistalPosition(currentDesiredState.getArmAngles().getDistalAngle());

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
