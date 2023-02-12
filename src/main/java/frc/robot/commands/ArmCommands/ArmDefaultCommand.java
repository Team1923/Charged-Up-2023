// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.interfaces.BetterLimelightInterface;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.util.StateHandler;
import frc.robot.util.StateVariables.ArmPositions;
import frc.robot.util.StateVariables.CurrentRobotDirection;
import frc.robot.util.StateVariables.IntakePositions;
import frc.robot.util.StateVariables.VerticalLocations;

public class ArmDefaultCommand extends CommandBase {
  /** Creates a new ArmDefaultCommand. */
  private ArmSubsystem armSubsystem;
  private StateHandler stateHandler = StateHandler.getInstance();
  private BetterLimelightInterface limelightInterface = BetterLimelightInterface.getInstance();
  private Timer timer;

  public ArmDefaultCommand(ArmSubsystem aSubsystem) {
    armSubsystem = aSubsystem;
    addRequirements(armSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ArmPositions currentDesiredState = stateHandler.getArmDesiredPosition();

    stateHandler.setTimeSinceReadyToScore(timer.get());

    switch (currentDesiredState) {
      case STOW:
        stateHandler.setIsArmMoving(false);
        if (stateHandler.getWantToScore() && stateHandler.getTimeSinceLastGripChange() > .3
            && stateHandler.getGripperEngaged()) {
          timer.start();

          if (timer.get() > 1 && stateHandler.getCurrentIntakePosition() == IntakePositions.STOW) {
            stateHandler.setArmDesiredState(ArmPositions.COBRA_FORWARD);
          }

        }
        break;
      case COBRA_FORWARD:
        stateHandler.setIsArmMoving(true);
        if (!stateHandler.getHoldInCobra() && stateHandler.getCurrentArmPosition() == ArmPositions.COBRA_FORWARD) {
          stateHandler.setArmDesiredState(stateHandler.getArmPositionFromScoringLocation());
        }
        if (!stateHandler.getWantToScore()) {
          stateHandler.setArmDesiredState(ArmPositions.STOW);
        }
        timer.reset();
        timer.stop();
        break;
      case COBRA_REVERSE:
        stateHandler.setIsArmMoving(false);
        if (stateHandler.getCurrentArmPosition() == ArmPositions.COBRA_REVERSE) {
          stateHandler.setArmDesiredState(ArmPositions.STOW);
        }
        timer.reset();
        timer.stop();
        break;
      case CONE_HIGH:
      case CONE_MID:
      case CUBE_HIGH:
      case CUBE_MID:
      case LOW:
        stateHandler.setIsArmMoving(false);
        if (!stateHandler.getWantToScore()
            || (!stateHandler.getGripperEngaged() && stateHandler.getTimeSinceLastGripChange() > .3)) {
          Timer.delay(0.3);
          stateHandler.setArmDesiredState(ArmPositions.COBRA_REVERSE);
          stateHandler.setWantToScore(false);
        }
        timer.reset();
        timer.stop();
      default:
        break;
    }

    if (stateHandler.getArmDesiredPosition().getArmAngles().getProximalAngle() < ArmConstants.minProximalPosition
        || stateHandler.getArmDesiredPosition().getArmAngles().getProximalAngle() > ArmConstants.maxProximalPosition) {
      armSubsystem.setProximalPosition(ArmPositions.STOW.getArmAngles().getProximalAngle());
    }

    if (stateHandler.getArmDesiredPosition().getArmAngles().getDistalAngle() < ArmConstants.minDistalPosition
        || stateHandler.getArmDesiredPosition().getArmAngles().getDistalAngle() > ArmConstants.maxDistalPosition) {
      armSubsystem.setDistalPosition(ArmPositions.STOW.getArmAngles().getDistalAngle());
    }

    if (stateHandler.getRobotDirection() == CurrentRobotDirection.RIGHT
        && stateHandler.getCurrentVerticalLocation() != VerticalLocations.RESET) {
      armSubsystem.setProximalPosition(stateHandler.getArmDesiredPosition().getArmAngles().getProximalAngle());
      armSubsystem.setDistalPosition(stateHandler.getArmDesiredPosition().getArmAngles().getDistalAngle());
    } else if (stateHandler.getRobotDirection() == CurrentRobotDirection.LEFT
        && stateHandler.getCurrentVerticalLocation() != VerticalLocations.RESET) { // redundant but leaving for
                                                                                   // readability
      armSubsystem.setProximalPosition(stateHandler.getArmDesiredPosition().getLeftArmAngles().getProximalAngle());
      armSubsystem.setDistalPosition(stateHandler.getArmDesiredPosition().getLeftArmAngles().getDistalAngle());

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
