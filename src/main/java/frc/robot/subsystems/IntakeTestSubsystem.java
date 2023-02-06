// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeTestSubsystem extends SubsystemBase {
  /** Creates a new IntakeTestSubsystem. */
  private WPI_TalonFX leftIntakeMotor = new WPI_TalonFX(0); //enter IDs
  private WPI_TalonFX rightIntakeMotor = new WPI_TalonFX(0);

  public IntakeTestSubsystem() {
    leftIntakeMotor.configFactoryDefault();
    rightIntakeMotor.configFactoryDefault();
  }

  public void runIntake(double stpt){
    leftIntakeMotor.set(ControlMode.PercentOutput, stpt);
    rightIntakeMotor.set(ControlMode.PercentOutput, -stpt);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
