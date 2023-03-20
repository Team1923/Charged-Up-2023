// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.util.StateHandler;
import frc.robot.util.StateVariables.IntakePositions;

public class IntakeDefaultCommand extends CommandBase {

  private IntakeSubsystem intake;
  private BooleanSupplier shootSupplier;
  private StateHandler stateHandler = StateHandler.getInstance();

  public IntakeDefaultCommand(IntakeSubsystem intake, BooleanSupplier s) {
    this.intake = intake;
    shootSupplier = s;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    setWheelSpeeds();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.disableMotionMagic();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public void setWheelSpeeds() {
    if (shootSupplier.getAsBoolean() || stateHandler.getAutoShootWheels()) {
      intake.setRawWheelSpeed(stateHandler.getShootingSpeedFromVerticalLocation());
    } else if (intake.getGamePieceSensor()) {
      intake.setRawWheelSpeed(IntakeConstants.intakeHoldSpeed);
    } else if (stateHandler.getCurrentIntakePosition() == IntakePositions.INTAKE) {
      intake.setRawWheelSpeed(IntakeConstants.cubeIntakeSpeed);
    } else {
      intake.setRawWheelSpeed(IntakeConstants.cubeIntakeSpeed);
    }
  }
}
