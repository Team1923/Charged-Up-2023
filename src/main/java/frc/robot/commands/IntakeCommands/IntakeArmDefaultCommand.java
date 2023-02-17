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
  private BooleanSupplier eject;

  /** Creates a new IntakeArmDefaultCommand. */
  public IntakeArmDefaultCommand(IntakeSubsystem i, DoubleSupplier driverRightJoystick, BooleanSupplier b) {
    intake = i;
    this.driverRightJoystick = driverRightJoystick;
    this.eject = b;
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
    setWheelSpeeds();
    intake.updateCurrentRollingAvg();
    IntakePositions currentDesiredState = stateHandler.getDesiredIntakePosition();

    /*
     * This switch staement takes in the currentDesired state of the intake, and for
     * each case,
     * sets the disered intake to the next in position. This works for going from
     * stow to intake
     * as well as intake to stow. A delay was added from Final Handoff to Stow
     */
    switch (currentDesiredState) {
      case STOW:
        if (stateHandler.getCurrentIntakePosition() == IntakePositions.STOW) {
          intake.setSolenoid(false);
        }
        break;
      case INTAKE:
        intake.setSolenoid(true);
        break;
      case HANDOFF_1:
        if (stateHandler.getCurrentIntakePosition() == IntakePositions.HANDOFF_1) {
          stateHandler.setDesiredIntakePosition(IntakePositions.HANDOFF_2);
        }
        break;
      case HANDOFF_2:
        if (stateHandler.getCurrentIntakePosition() == IntakePositions.HANDOFF_2) {
          if (!intake.getAverageCurrentAboveThreshold(7)) {
            stateHandler.setDesiredIntakePosition(IntakePositions.STOW);
          } else {
            stateHandler.setDesiredIntakePosition(IntakePositions.FINAL_HANDOFF);
          }
        }
        break;
      case FINAL_HANDOFF:
        if (stateHandler.getCurrentIntakePosition() == IntakePositions.FINAL_HANDOFF
            && stateHandler.getTimeSinceReadyToSore() > .5) {
          intake.setSolenoid(false);
        }
        if (stateHandler.getCurrentIntakePosition() == IntakePositions.FINAL_HANDOFF
            && stateHandler.getTimeSinceReadyToSore() > .75) {
          stateHandler.setDesiredIntakePosition(IntakePositions.STOW);
        }
        break;
      case REVERSE_HANDOFF_2:
        if (stateHandler.getCurrentIntakePosition() == IntakePositions.REVERSE_HANDOFF_2) {
          stateHandler.setDesiredIntakePosition(IntakePositions.INTAKE);

        }
        intake.setSolenoid(true);
        break;
      case REVERSE_HANDOFF_1:
        if (stateHandler.getCurrentIntakePosition() == IntakePositions.REVERSE_HANDOFF_1) {
          stateHandler.setDesiredIntakePosition(IntakePositions.REVERSE_HANDOFF_2);
        }
        break;
      default:
        break;
    }

    currentDesiredState = stateHandler.getDesiredIntakePosition();

    intake.setIntakeProximalPosition(currentDesiredState.getArmAngles().getProximalAngle());
    intake.setIntakeDistalPosition(currentDesiredState.getArmAngles().getDistalAngle());

  }

  public void setWheelSpeeds() {
    if (eject.getAsBoolean()) {
      intake.setRawWheelSpeed(-0.2);
    }
    if (stateHandler.getIsArmMoving()) {
      intake.setRawWheelSpeed(-0.1);
    } else if (intake.getAverageCurrentAboveThreshold(20)) {
      intake.setRawWheelSpeed(0.1);
    } else if (stateHandler.getGamePieceMode() == GamePieceMode.CONE && driverRightJoystick.getAsDouble() > 0.2) {
      intake.setRawWheelSpeed(IntakeConstants.coneIntakeSpeed);
    } else if (stateHandler.getGamePieceMode() == GamePieceMode.CUBE && driverRightJoystick.getAsDouble() > 0.2) {
      intake.setRawWheelSpeed(IntakeConstants.cubeIntakeSpeed);
    } else {
      intake.setRawWheelSpeed(0.1);
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
