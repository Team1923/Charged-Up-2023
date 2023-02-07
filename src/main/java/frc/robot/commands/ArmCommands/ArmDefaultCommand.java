// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.interfaces.BetterLimelightInterface;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.util.StateHandler;
import frc.robot.util.StateVariables.ArmPositions;
import frc.robot.util.StateVariables.CurrentRobotDirection;
import frc.robot.util.StateVariables.GamePieceMode;
import frc.robot.util.StateVariables.IntakePositions;

public class ArmDefaultCommand extends CommandBase {
  /** Creates a new ArmDefaultCommand. */
  private ArmSubsystem armSubsystem;
  private StateHandler stateHandler = StateHandler.getInstance();
  private BetterLimelightInterface limelightInterface = BetterLimelightInterface.getInstance();
  private double joystickPOV;

  public ArmDefaultCommand(ArmSubsystem aSubsystem, double joystickPOV) {
    armSubsystem = aSubsystem;
    this.joystickPOV = joystickPOV;
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
    ArmPositions currentDesiredState = stateHandler.getArmDesiredPosition();

    switch (currentDesiredState) {
      case STOW:
        if(stateHandler.getWantToScore() && stateHandler.getTimeSinceLastGripChange() > .3 && stateHandler.getGripperEngaged()) {
          stateHandler.setArmDesiredState(ArmPositions.COBRA_FORWARD);
        }
        break;
      case COBRA_FORWARD:
        if(!stateHandler.getHoldInCobra() && stateHandler.getCurrentArmPosition() == ArmPositions.COBRA_FORWARD) {
          stateHandler.setArmDesiredState(stateHandler.getArmPositionFromScoringLocation());
        }
        break;
      case COBRA_REVERSE:
        if(stateHandler.getCurrentArmPosition() == ArmPositions.COBRA_FORWARD) {
          stateHandler.setArmDesiredState(ArmPositions.STOW);
        }
        break;
      case CONE_HIGH:
      case CONE_MID:
      case CUBE_HIGH:
      case CUBE_MID:
      case LOW:
        if(!stateHandler.getWantToScore() || (!stateHandler.getGripperEngaged() && stateHandler.getTimeSinceLastGripChange() > .3)) {
          stateHandler.setArmDesiredState(ArmPositions.COBRA_REVERSE);
        }
      default:
        break;
    }

    if (stateHandler.getRobotDirection() == CurrentRobotDirection.RIGHT) {
      armSubsystem.setProximalPosition(stateHandler.getArmDesiredPosition().getArmAngles().getProximalAngle());
      armSubsystem.setDistalPosition(stateHandler.getArmDesiredPosition().getArmAngles().getDistalAngle());
    } else {
      /* The proximal angle for cone HIGH is positive. slightly different conversion math required. */
      armSubsystem.setProximalPosition(stateHandler.getArmDesiredPosition().getReflectedArmAngles().getProximalAngle());
      if (stateHandler.getArmDesiredPosition() == ArmPositions.CONE_HIGH) {
        armSubsystem.setDistalPosition((2 * Math.PI) - stateHandler.getArmDesiredPosition().getArmAngles().getDistalAngle());
      } else {
        armSubsystem.setDistalPosition(stateHandler.getArmDesiredPosition().getReflectedArmAngles().getDistalAngle());
      }
      
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
