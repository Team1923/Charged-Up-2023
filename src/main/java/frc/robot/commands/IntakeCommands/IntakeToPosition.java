// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.util.StateVariables.IntakePositions;

public class IntakeToPosition extends CommandBase {
  /** Creates a new IntakeToPosition. */
  private IntakeSubsystem intakeSubsystem;
  private BooleanSupplier test;
  private BooleanSupplier otherTest;
  public IntakeToPosition(IntakeSubsystem i, BooleanSupplier test, BooleanSupplier otherTest) {
    this.intakeSubsystem = i;
    this.test = test;
    this.otherTest = otherTest;
    addRequirements(intakeSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(test.getAsBoolean()){
      if(otherTest.getAsBoolean()){
      //   intakeSubsystem.setIntakeProximalPosition(IntakePositions.CONE_HANDOFF.getArmAngles().getProximalAngle());
      //   intakeSubsystem.setIntakeDistalPosition(IntakePositions.CONE_HANDOFF.getArmAngles().getDistalAngle());
      // } else{
      //   intakeSubsystem.setIntakeProximalPosition(IntakePositions.TEST_1.getArmAngles().getProximalAngle());
      //   intakeSubsystem.setIntakeDistalPosition(IntakePositions.TEST_1.getArmAngles().getDistalAngle());
      }
    } else{
      intakeSubsystem.setIntakeProximalPosition(IntakePositions.INTAKE.getArmAngles().getProximalAngle());
        intakeSubsystem.setIntakeDistalPosition(IntakePositions.INTAKE.getArmAngles().getDistalAngle());
    }

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setIntakeProximalPosition(IntakePositions.INTAKE.getArmAngles().getProximalAngle());
    intakeSubsystem.setIntakeDistalPosition(IntakePositions.INTAKE.getArmAngles().getDistalAngle());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
