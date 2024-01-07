// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
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
  private Timer timer;
  private CurrentRobotDirection currentRobotDirection;

  public ArmDefaultCommand(ArmSubsystem aSubsystem) {
    armSubsystem = aSubsystem;
    addRequirements(armSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
    timer = new Timer();
    currentRobotDirection = stateHandler.getRobotDirection();
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

    // takes each case of the arm's position in order to determine where it should
    // go next, and preps the arm to actually move there
    switch (currentDesiredState) {
      case STOW:
        currentRobotDirection = stateHandler.getRobotDirection();
        stateHandler.setIsArmMoving(false);
        /*
         * if:
         * we want to score,
         * the time since we last closed our gripper is greater than .3 seconds, and
         * the gripper is engaged
         * we can start the timer
         */
        if (stateHandler.getWantToScore() && stateHandler.getTimeSinceLastGripChange() > .3
            && stateHandler.getGripperEngaged()) {
          timer.start();

          // if the timer has been running for more than a second and the current intake
          // position is stow,
          // we can set the desired arm state to cobra forward
          if (timer.get() > 0.75 && (stateHandler.getCurrentIntakePosition() == IntakePositions.STOW
              || stateHandler.getCurrentIntakePosition() == IntakePositions.FINAL_HANDOFF)) {
            stateHandler.setArmDesiredState(ArmPositions.COBRA_FORWARD);
          } 

        }
        break;

      case FEED:
        if (stateHandler.getCurrentArmPosition() == ArmPositions.FEED) {
          stateHandler.setDesiredIntakePosition(IntakePositions.EJECT);
        }
        break;
      case COBRA_FORWARD:
        stateHandler.setIsArmMoving(true);
        stateHandler.setHasGamePiece(false);

        if (stateHandler.getCurrentArmPosition() == ArmPositions.COBRA_FORWARD) {
          if (stateHandler.getCurrentIntakePosition() == IntakePositions.EJECT &&
              stateHandler.getDesiredIntakePosition() == IntakePositions.EJECT) {
            stateHandler.setDesiredIntakePosition(IntakePositions.STOW);
          }
        }
        /*
         * if:
         * the arm is not being held in the cobra position, and
         * we are in the cobra forward position,
         * then we can set the desired state of the arm based on where we want to score
         */
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
        // if we are currently in the cobra reverse position, then set the desired arm
        // state to stow
        if (stateHandler.getCurrentArmPosition() == ArmPositions.COBRA_REVERSE) {
          stateHandler.setArmDesiredState(ArmPositions.STOW);
        }
        timer.reset();
        timer.stop();
        break;
      case INVERTED_COBRA_REVERSE:
        stateHandler.setIsArmMoving(false);
        // if we are currently in the cobra reverse position, then set the desired arm
        // state to stow
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

    if (currentRobotDirection == CurrentRobotDirection.RIGHT) {
      armSubsystem.setProximalPosition(stateHandler.getArmDesiredPosition().getArmAngles().getProximalAngle());
      armSubsystem.setDistalPosition(stateHandler.getArmDesiredPosition().getArmAngles().getDistalAngle());
    } else if (currentRobotDirection == CurrentRobotDirection.LEFT) {
      armSubsystem.setProximalPosition(stateHandler.getArmDesiredPosition().getLeftArmAngles().getProximalAngle());
      armSubsystem.setDistalPosition(stateHandler.getArmDesiredPosition().getLeftArmAngles().getDistalAngle());
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.disableMotionMagic();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
