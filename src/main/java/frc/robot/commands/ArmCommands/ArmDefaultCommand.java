// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj.PS4Controller;
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
        if (stateHandler.getGripperEngaged()) {
          stateHandler.setArmDesiredState(ArmPositions.CLEAR);
        } else if (limelightInterface.hasScoringTarget()) {
          stateHandler.setArmDesiredState(ArmPositions.COBRA);
        }
        break;
      case CLEAR:
        if (stateHandler.getCurrentIntakePosition() == IntakePositions.STOW) {
          stateHandler.setArmDesiredState(ArmPositions.STOW);
        }
        break;
      case COBRA:
        if (joystickPOV == 0) { // up arrow
          if (stateHandler.getGamePieceMode() == GamePieceMode.CONE) {
            stateHandler.setArmDesiredState(ArmPositions.CONE_HIGH);
          } else {
            stateHandler.setArmDesiredState(ArmPositions.CUBE_HIGH);
          }
        } else if (joystickPOV == 270) {
          if (stateHandler.getGamePieceMode() == GamePieceMode.CONE) {
            stateHandler.setArmDesiredState(ArmPositions.CONE_MID);
          } else {
            stateHandler.setArmDesiredState(ArmPositions.CUBE_MID);
          }
        } else if (joystickPOV == 180) {
          stateHandler.setArmDesiredState(ArmPositions.LOW);
        } else if (!stateHandler.getGripperEngaged()) {
          stateHandler.setArmDesiredState(ArmPositions.STOW);
        }
        break;
      case CONE_HIGH:
      case CONE_MID:
      case CUBE_HIGH:
      case CUBE_MID:
      case LOW:
        if (!stateHandler.getGripperEngaged()) {
          stateHandler.setArmDesiredState(ArmPositions.COBRA);
        }
        break;
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
