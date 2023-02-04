// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.util.StateHandler;
import frc.robot.util.StateVariables.ArmPositions;
import frc.robot.util.StateVariables.CurrentRobotDirection;

public class OldArmDefaultCommand extends CommandBase {
  /** Creates a new ArmDefaultCommand. */
  private ArmSubsystem armSubsystem;
  private StateHandler stateHandler = StateHandler.getInstance();

  public OldArmDefaultCommand(ArmSubsystem aSubsystem) {
    armSubsystem = aSubsystem;
    addRequirements(armSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double proximalSetpoint = stateHandler.getArmDesiredPosition().getArmAngles().getProximalAngle();
    if (stateHandler.getRobotDirection() == CurrentRobotDirection.LEFT) {
      proximalSetpoint = stateHandler.getArmDesiredPosition().getReflectedArmAngles().getProximalAngle();
    }
    armSubsystem.setProximalPosition(proximalSetpoint);

    double distalSetpoint = stateHandler.getArmDesiredPosition().getArmAngles().getDistalAngle();
    if (stateHandler.getRobotDirection() == CurrentRobotDirection.LEFT) {
      distalSetpoint = stateHandler.getArmDesiredPosition().getReflectedArmAngles().getDistalAngle();
    }
    armSubsystem.setDistalPosition(distalSetpoint);

    // To avoid putting the rest of the code inside an if-statement, I will check if
    // the object is null
    // and return nothing to exit this iteration of the execute loop. Any code that
    // I would normally
    // put inside the if statement I can now put after it, as the loop would have
    // exited if the if statement
    // was true in the last check.
    ArmPositions nextInSequence = stateHandler.getArmDesiredPosition().getNextInSequence();

    if (nextInSequence == null) {
      return;
    }

    double proximalError = Math.abs(armSubsystem.getProximalPosition() - proximalSetpoint);
    double distalError = Math.abs(armSubsystem.getDistalPosition() - distalSetpoint);

    double triggerThresholdRadians = stateHandler.getArmDesiredPosition().getThresholdRadians();

    if (proximalError < triggerThresholdRadians && distalError < triggerThresholdRadians) {
      stateHandler.setArmDesiredState(nextInSequence);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
