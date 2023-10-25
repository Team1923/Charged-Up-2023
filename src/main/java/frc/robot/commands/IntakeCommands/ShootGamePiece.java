// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Encoder.IndexingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.util.StateHandler;
import frc.robot.util.StateVariables.IntakePositions;
import frc.robot.util.StateVariables.IntakeWheelSpeeds;
import frc.robot.util.StateVariables.VerticalLocations;

public class ShootGamePiece extends CommandBase {
  /** Creates a new ShootGamePiece. */

  StateHandler stateHandler = StateHandler.getInstance();

  IntakeWheelSpeeds desiredShootSpeed = IntakeWheelSpeeds.GRIP;

  Timer plopTimer;

  public ShootGamePiece() {
    plopTimer = new Timer();
    plopTimer.stop();
    plopTimer.reset();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    plopTimer.stop();
    plopTimer.reset();

    if(!(stateHandler.getDesiredIntakePosition() == IntakePositions.SHOOT_FRONT_HIGH)){
      if(stateHandler.getCurrentVerticalLocation() == VerticalLocations.LOW) {
        desiredShootSpeed = IntakeWheelSpeeds.SHOOT_LOW;
      } else if(stateHandler.getCurrentVerticalLocation() == VerticalLocations.MID) {
        desiredShootSpeed = IntakeWheelSpeeds.SHOOT_MID;
      } else {
        desiredShootSpeed = IntakeWheelSpeeds.SHOOT_HIGH;
      }
  
      if(desiredShootSpeed == IntakeWheelSpeeds.SHOOT_HIGH  || desiredShootSpeed == IntakeWheelSpeeds.SHOOT_MID)  {
        plopTimer.start();
        stateHandler.setDesiredIntakePosition(IntakePositions.PLOP_SHOT);
      }
    } 

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // SmartDashboard.putNumber("HIGH TIMER?", plopTimer.get());
    if(!(stateHandler.getDesiredIntakePosition() == IntakePositions.SHOOT_FRONT_HIGH)){
    if((desiredShootSpeed == IntakeWheelSpeeds.SHOOT_HIGH || desiredShootSpeed == IntakeWheelSpeeds.SHOOT_MID) && plopTimer.get() < 1) {
      stateHandler.setDesiredIntakeWheelSpeed(IntakeWheelSpeeds.GRIP);
    } else if((desiredShootSpeed == IntakeWheelSpeeds.SHOOT_HIGH || desiredShootSpeed == IntakeWheelSpeeds.SHOOT_MID) && plopTimer.get() > 1) {
      stateHandler.setDesiredIntakeWheelSpeed(desiredShootSpeed == IntakeWheelSpeeds.SHOOT_HIGH ? IntakeWheelSpeeds.PLOP_HIGH : IntakeWheelSpeeds.PLOP_MID);
    } else {
      stateHandler.setDesiredIntakeWheelSpeed(desiredShootSpeed);
    }
  }
  else{
    switch (stateHandler.getCurrentVerticalLocation()){
      case HIGH:
        stateHandler.setDesiredIntakeWheelSpeed(IntakeWheelSpeeds.SHOOT_FRONT_HIGH);
        break;
      case MID:
        stateHandler.setDesiredIntakeWheelSpeed(IntakeWheelSpeeds.SHOOT_FRONT_MID);
        break;
      case LOW:
        stateHandler.setDesiredIntakeWheelSpeed(IntakeWheelSpeeds.SHOOT_FRONT_LOW);
        break;
      default:
        break;

    }   
  }
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    stateHandler.setDesiredIntakeWheelSpeed(IntakeWheelSpeeds.GRIP);
    if(stateHandler.getDesiredIntakePosition() == IntakePositions.PLOP_SHOT) {
      stateHandler.setDesiredIntakePosition(IntakePositions.SHOOT_TALL);
    }

    stateHandler.setReadyToScore(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
