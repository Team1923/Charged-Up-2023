// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class SqueezeIntakeCommand extends CommandBase {

  private IntakeSubsystem intake;
  private Timer timer;


  /** Creates a new SqueezeIntakeCommand. */
  public SqueezeIntakeCommand(IntakeSubsystem i) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = i;
    this.timer = new Timer();
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(timer.get() < 0.25) {
      intake.setRawWheelSpeed(1);
    } else {
      intake.setRawWheelSpeed(-1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    intake.stopWheels();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > 2.25;
  }
}
