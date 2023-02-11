// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Scoring;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.util.StateHandler;

public class ManualScore extends CommandBase {
  private StateHandler stateHandler;
  /** Creates a new ManualScore. */
  public ManualScore() {
    stateHandler = StateHandler.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //System.out.println("test1");
    stateHandler.setWantToScore(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //System.out.println("test2");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //System.out.println("test3");


  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
