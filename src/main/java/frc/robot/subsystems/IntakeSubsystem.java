// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FalconConstants;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private WPI_TalonFX intakeProximalMotor = new WPI_TalonFX(IntakeConstants.intakeProximalID);
  private WPI_TalonFX intakeDistalMotor = new WPI_TalonFX(IntakeConstants.intakeDistalID);
  private DutyCycleEncoder intakeProximalEncoder = new DutyCycleEncoder(IntakeConstants.intakeProximalAbsoluteEncoderID);
  private DutyCycleEncoder intakeDistalEncoder = new DutyCycleEncoder(IntakeConstants.intakeDistalAbsoluteEncoderID);
  
  public IntakeSubsystem() {
    intakeProximalMotor.configFactoryDefault();
    intakeDistalMotor.configFactoryDefault();

    intakeProximalMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, FalconConstants.timeoutMs);
    intakeDistalMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, FalconConstants.timeoutMs);

    intakeProximalMotor.config_kP(0, IntakeConstants.intakeProximalkP, FalconConstants.timeoutMs);
    intakeProximalMotor.config_kI(0, IntakeConstants.intakeProximalkI, FalconConstants.timeoutMs);
    intakeProximalMotor.config_kD(0, IntakeConstants.intakeProximalkD, FalconConstants.timeoutMs);

    intakeDistalMotor.config_kP(0, IntakeConstants.intakeDistalkP, FalconConstants.timeoutMs);
    intakeDistalMotor.config_kP(0, IntakeConstants.intakeDistalkI, FalconConstants.timeoutMs);
    intakeDistalMotor.config_kP(0, IntakeConstants.intakeDistalkD, FalconConstants.timeoutMs);

    intakeProximalMotor.configMotionCruiseVelocity(IntakeConstants.maxIntakeProximalVel);
    intakeProximalMotor.configMotionAcceleration(IntakeConstants.maxIntakeProximalAccel);
    intakeDistalMotor.configMotionCruiseVelocity(IntakeConstants.maxIntakeDistalVel);
    intakeDistalMotor.configMotionAcceleration(IntakeConstants.maxIntakeDistalAccel);

    intakeProximalMotor.setNeutralMode(NeutralMode.Brake);
    intakeDistalMotor.setNeutralMode(NeutralMode.Brake);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
