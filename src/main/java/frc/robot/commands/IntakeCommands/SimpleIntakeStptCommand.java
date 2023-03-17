// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.util.StateHandler;
import frc.robot.util.StateVariables.IntakePositions;

public class SimpleIntakeStptCommand extends CommandBase {
  /** Creates a new SimpleIntakeStptCommand. */
  private IntakeSubsystem intake;
  private IntakePositions desiredIntakePosition;
  public SimpleIntakeStptCommand(IntakeSubsystem intake, IntakePositions desiredIntakePosition) {
    this.intake = intake;
    this.desiredIntakePosition = desiredIntakePosition;
    addRequirements(intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setIntakePosition(desiredIntakePosition.getArmAngles().getAngle());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return desiredIntakePosition.getArmAngles().getAngle() == intake.getIntakeArmPosition();
  }
}
