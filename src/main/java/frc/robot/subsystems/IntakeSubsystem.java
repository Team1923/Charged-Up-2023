// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase implements IntakeIO{
  /** Creates a new IntakeSubsystem. */

  private WPI_TalonFX intakeMotor = new WPI_TalonFX(0);
  private final IntakeIOInputs inputs = new IntakeIOInputs();

  public IntakeSubsystem() {
    //stuff we usually do in a subsystem
    intakeMotor.configFactoryDefault();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateInputs(inputs);
    Logger.getInstance().processInputs("Intake", inputs);
  }


  //now we actually have to update the inputs and implement the method
  @Override
  public void updateInputs(IntakeIOInputs inputs){
      inputs.speed = 0.9;
      inputs.position = intakeMotor.getSelectedSensorPosition();
      inputs.isRunning = intakeMotor.getMotorOutputVoltage() > 0.2;
  }

  public void runIntake(){

  }
}
