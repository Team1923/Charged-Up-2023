// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.util.StateHandler;

public class IntakeArmDefaultCommand extends CommandBase {

  private IntakeArmSubsystem intake;
  private StateHandler stateHandler;
  /** Creates a new IntakeArmDefaultCommand. */
  public IntakeArmDefaultCommand(IntakeArmSubsystem i) {
    intake = i;
    addRequirements(intake);

    this.stateHandler = StateHandler.getInstance();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
