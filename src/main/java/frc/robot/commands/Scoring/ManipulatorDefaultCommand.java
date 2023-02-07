// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Scoring;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.util.StateHandler;

public class ManipulatorDefaultCommand extends CommandBase {

  private ManipulatorSubsystem gripper;
  private boolean latch = false;
  private StateHandler stateHandler;
  private boolean lastGripperValue;

  /** Creates a new ManipulatorDefaultCommand. */
  public ManipulatorDefaultCommand(ManipulatorSubsystem gripper) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.gripper = gripper;
    addRequirements(gripper);
    stateHandler = StateHandler.getInstance();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lastGripperValue = gripper.get();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean reset = StateHandler.getInstance().getResetManipulator();

    if (reset) {
      latch = false;
      gripper.set(false);
      stateHandler.setGripperEngaged(false);
    } else if (StateHandler.getInstance().readyToClose() && !latch) {
      latch = true;
      gripper.set(true);
      stateHandler.setGripperEngaged(true);
    } else if (!latch) {
      gripper.set(false);
      stateHandler.setGripperEngaged(false);
    }

    boolean currentGripperValue = gripper.get();

    if(lastGripperValue != currentGripperValue) {
      stateHandler.setTimeSinceLastGripChange();
    }
    
    lastGripperValue = currentGripperValue;
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
