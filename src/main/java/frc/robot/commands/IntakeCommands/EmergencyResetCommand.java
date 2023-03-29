// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.util.StateHandler;
import frc.robot.util.StateVariables.IntakePositions;

public class EmergencyResetCommand extends CommandBase {
  /** Creates a new EmergencyResetCommand. */
  private static EmergencyResetCommand emergencyResetCommand;

  public static synchronized EmergencyResetCommand getInstance(IntakeSubsystem intake) {
    if(emergencyResetCommand == null) {
      emergencyResetCommand = new EmergencyResetCommand(intake);
    }

    return emergencyResetCommand;
  }

  private IntakeSubsystem intakeSubsystem;

  public EmergencyResetCommand(IntakeSubsystem intake) {
    intakeSubsystem = intake;
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    StateHandler.getInstance().setDesiredIntakePosition(IntakePositions.SHOOT_TALL);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSubsystem.setRawIntakeArmSpeed(.1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.emergencyResetIntakePosition();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
