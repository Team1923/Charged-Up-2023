// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.util.StateHandler;
import frc.robot.util.StateVariables.CurrentRobotDirection;

public class ArmDefaultCommand extends CommandBase {
  /** Creates a new ArmDefaultCommand. */
  private ArmSubsystem armSubsystem;
  private StateHandler stateHandler = StateHandler.getInstance();

  public ArmDefaultCommand(ArmSubsystem aSubsystem) {
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
    double proximalSetpoint = stateHandler.getArmPositions().getArmAngles().getProximalAngle();
    if (stateHandler.getRobotDirection() == CurrentRobotDirection.LEFT) {
      proximalSetpoint = stateHandler.getArmPositions().getReflectedArmAngles().getProximalAngle();
    }
    armSubsystem.setProximalPosition(proximalSetpoint);

    double distalSetpoint = stateHandler.getArmPositions().getArmAngles().getDistalAngle();
    if (stateHandler.getRobotDirection() == CurrentRobotDirection.LEFT) {
      distalSetpoint = stateHandler.getArmPositions().getReflectedArmAngles().getDistalAngle();
    }
    armSubsystem.setDistalPosition(distalSetpoint);
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
