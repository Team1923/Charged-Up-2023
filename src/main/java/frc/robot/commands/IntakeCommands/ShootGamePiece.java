// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Encoder.IndexingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.SwerveCommands.LockWheels;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.StateHandler;
import frc.robot.util.StateVariables.IntakePositions;
import frc.robot.util.StateVariables.IntakeWheelSpeeds;
import frc.robot.util.StateVariables.VerticalLocations;

public class ShootGamePiece extends CommandBase {
  /** Creates a new ShootGamePiece. */

  StateHandler stateHandler = StateHandler.getInstance();

  IntakeWheelSpeeds desiredShootSpeed = IntakeWheelSpeeds.GRIP;

  Timer plopTimer;
  Timer frontTimer;

  private SwerveSubsystem swerveSubsystem;

  public ShootGamePiece(SwerveSubsystem s) {
    plopTimer = new Timer();
    frontTimer = new Timer();
    plopTimer.stop();
    plopTimer.reset();
    frontTimer.stop();
    frontTimer.reset();
    this.swerveSubsystem = s;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    plopTimer.stop();
    plopTimer.reset();

    frontTimer.stop();
    frontTimer.reset();

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
    else{
        //here our position is shoot front high, still question of wheelspeeds being set
        frontTimer.start();
      
    }
    

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // SmartDashboard.putNumber("HIGH TIMER?", plopTimer.get());
    if(!(stateHandler.getDesiredIntakePosition() == IntakePositions.SHOOT_FRONT_HIGH)){
    if((desiredShootSpeed == IntakeWheelSpeeds.SHOOT_HIGH || desiredShootSpeed == IntakeWheelSpeeds.SHOOT_MID) && plopTimer.get() < 1.5) {
      stateHandler.setDesiredIntakeWheelSpeed(IntakeWheelSpeeds.GRIP);
    } else if((desiredShootSpeed == IntakeWheelSpeeds.SHOOT_HIGH || desiredShootSpeed == IntakeWheelSpeeds.SHOOT_MID) && plopTimer.get() > 1.5) {
      stateHandler.setDesiredIntakeWheelSpeed(desiredShootSpeed == IntakeWheelSpeeds.SHOOT_HIGH ? IntakeWheelSpeeds.PLOP_HIGH : IntakeWheelSpeeds.PLOP_MID);
    } else {
      stateHandler.setDesiredIntakeWheelSpeed(desiredShootSpeed);
    }
  }
  else{
    if (stateHandler.getCurrentVerticalLocation() == VerticalLocations.HIGH || stateHandler.getCurrentVerticalLocation() == VerticalLocations.MID ){
        stateHandler.setLockWheels(true);
        CommandScheduler.getInstance().schedule(new LockWheels(swerveSubsystem));
        StateHandler.getInstance().setWantToBeHappy(true);
      }
    switch (stateHandler.getCurrentVerticalLocation()){
      case HIGH:
        if(frontTimer.get() > 0.5){
          stateHandler.setDesiredIntakeWheelSpeed(IntakeWheelSpeeds.SHOOT_FRONT_HIGH);
        }
        break;
      case MID:
        if(frontTimer.get() > 0.5){
          stateHandler.setDesiredIntakeWheelSpeed(IntakeWheelSpeeds.SHOOT_FRONT_MID);
        }
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
    
    stateHandler.setLockWheels(false);
    stateHandler.setReadyToScore(false);
    StateHandler.getInstance().setWantToBeHappy(false);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
