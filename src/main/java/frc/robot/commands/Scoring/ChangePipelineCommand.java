// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Scoring;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.interfaces.LimelightInterface;
import frc.robot.interfaces.LimelightInterface.Limelight;

public class ChangePipelineCommand extends CommandBase {
  /** Creates a new ChangePipelineCommand. */
  int pipeid;
  Limelight limelight;
  public ChangePipelineCommand(int id, Limelight limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.pipeid = id;
    this.limelight = limelight;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LimelightInterface.getInstance().setPipeline(pipeid, limelight);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    LimelightInterface.getInstance().setPipeline(0, limelight);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
