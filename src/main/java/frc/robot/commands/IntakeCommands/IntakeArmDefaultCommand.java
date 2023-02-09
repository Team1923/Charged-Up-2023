// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

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
  private DoubleSupplier driverRightJoystick;

  /** Creates a new IntakeArmDefaultCommand. */
  public IntakeArmDefaultCommand(IntakeSubsystem i, DoubleSupplier driverRightJoystick) {
    intake = i;
    this.driverRightJoystick = driverRightJoystick;
    addRequirements(intake);

    this.stateHandler = StateHandler.getInstance();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setSolenoid(false);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (driverRightJoystick.getAsDouble() > 0.2) {
      intake.setRawWheelSpeed(0.5);
    } else {
      intake.setRawWheelSpeed(.1);
    }
    IntakePositions currentDesiredState = stateHandler.getDesiredIntakePosition();

    switch (currentDesiredState) {
      case STOW:
        intake.setSolenoid(false);
      case INTAKE:
        if (intake.intakeHasGamePiece()) {
          intake.setSolenoid(true);
          stateHandler.setDesiredIntakePosition(IntakePositions.HANDOFF_1);
        }
        break;
      case HANDOFF_1:
        if (stateHandler.getCurrentIntakePosition() == IntakePositions.HANDOFF_1) {
          stateHandler.setDesiredIntakePosition(IntakePositions.HANDOFF_2);
        }
        break;
      case HANDOFF_2:
        if (intake.intakeHasGamePiece() && stateHandler.getCurrentIntakePosition() == IntakePositions.HANDOFF_2) {
          stateHandler.setDesiredIntakePosition(IntakePositions.FINAL_HANDOFF);
        }
        if (!intake.intakeHasGamePiece() && stateHandler.getCurrentIntakePosition() == IntakePositions.HANDOFF_2) {
          stateHandler.setDesiredIntakePosition(IntakePositions.STOW);
        }
        break;
      case FINAL_HANDOFF:
        if (stateHandler.getCurrentIntakePosition() == IntakePositions.FINAL_HANDOFF) {
          intake.setSolenoid(false);
        }
        if (stateHandler.getCurrentArmPosition() == ArmPositions.COBRA_FORWARD) {
          stateHandler.setDesiredIntakePosition(IntakePositions.STOW);
        }
        break;
      case REVERSE_HANDOFF_2:
        if (stateHandler.getCurrentIntakePosition() == IntakePositions.REVERSE_HANDOFF_2) {
          stateHandler.setDesiredIntakePosition(IntakePositions.REVERSE_HANDOFF_1);
          intake.setSolenoid(true);
        }
        break;
      case REVERSE_HANDOFF_1:
        if (stateHandler.getCurrentIntakePosition() == IntakePositions.REVERSE_HANDOFF_1) {
          stateHandler.setDesiredIntakePosition(IntakePositions.INTAKE);
        }
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
