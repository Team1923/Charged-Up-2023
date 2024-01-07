// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class ConfirmBalanceCommand extends CommandBase {

  private SwerveSubsystem swerve;
  private PIDController pitchController = new PIDController(3, 0, 0.02);

  private Timer timer;

  /** Creates a new ConfirmBalanceCommand. */
  public ConfirmBalanceCommand(SwerveSubsystem s) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = s;
    timer = new Timer();

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.stop();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xOutput = -pitchController.calculate(Math.sin(swerve.getPitch().getRadians()), 0);
    SmartDashboard.putNumber("X OUTPUT", xOutput);
    Translation2d output = new Translation2d(xOutput, 0);
    swerve.drive(output, 0, true, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("ENDED WITH GYRO", true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double error = Math.abs(swerve.getPitch().getDegrees());
    if(error < 2) {
      timer.start();
    } else {
      timer.reset();
      timer.stop();
    }
    return timer.get() > 0.2;
  }
}
