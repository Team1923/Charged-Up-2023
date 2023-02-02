// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FalconConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.util.StateHandler;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private WPI_TalonFX intakeProximalMotor = new WPI_TalonFX(IntakeConstants.intakeProximalID);
  private WPI_TalonFX intakeDistalMotor = new WPI_TalonFX(IntakeConstants.intakeDistalID);
  private WPI_TalonFX intakeWheelMotor = new WPI_TalonFX(IntakeConstants.intakeWheelID);

  private DutyCycleEncoder intakeProximalEncoder = new DutyCycleEncoder(
      IntakeConstants.intakeProximalAbsoluteEncoderID);
  private DutyCycleEncoder intakeDistalEncoder = new DutyCycleEncoder(IntakeConstants.intakeDistalAbsoluteEncoderID);

  private StateHandler stateHandler = StateHandler.getInstance();

  public IntakeSubsystem() {
    intakeProximalMotor.configFactoryDefault();
    intakeDistalMotor.configFactoryDefault();
    intakeWheelMotor.configFactoryDefault();

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
    
    resetIntakeProximalPosition();
    resetIntakeDistalPosition();
  }

  public void resetIntakeProximalPosition(){
    intakeProximalMotor.setSelectedSensorPosition((getIntakeProximalAbsoluteEncoderRads() - 
    IntakeConstants.intakeProximalEncoderHardstop + 
    IntakeConstants.intakeProximalHardstop) * IntakeConstants.intakeProximalRadsToTicks);
  }

  public void resetIntakeDistalPosition(){
    intakeDistalMotor.setSelectedSensorPosition((getIntakeDistalAbsoluteEncoderRads() - 
    IntakeConstants.intakeDistalEncoderHardstop + 
    IntakeConstants.intakeDistalHardstop) * IntakeConstants.intakeDistalRadsToTicks);
  }

  public void setIntakeProximalPosition(double proximalAngle){
    intakeProximalMotor.set(ControlMode.MotionMagic, proximalAngle * IntakeConstants.intakeProximalRadsToTicks,
        DemandType.ArbitraryFeedForward, calculateIntakeProximalFeedforward());
  }

  public void setIntakeDistalPosition(double distalAngle){
    intakeDistalMotor.set(ControlMode.MotionMagic, distalAngle * IntakeConstants.intakeDistalRadsToTicks,
      DemandType.ArbitraryFeedForward, calculateIntakeDistalFeedforward());
  }

  public double getIntakeProximalPosition(){
    return (intakeProximalMotor.getSelectedSensorPosition() * IntakeConstants.intakeProximalTicksToRad)
        + IntakeConstants.kIntakeProximalOffsetRads;
  }

  public double getIntakeDistalPosition(){
    return (intakeDistalMotor.getSelectedSensorPosition() * IntakeConstants.intakeDistalTicksToRad)
        + IntakeConstants.kIntakeDistalOffsetRads;
  }

  public double getIntakeProximalAbsoluteEncoderRads(){
    return intakeProximalEncoder.getAbsolutePosition() * IntakeConstants.intakeProximalAbsoluteEncoderToRadians;
  }

  public double getIntakeDistalAbsoluteEncoderRads(){
    return intakeDistalEncoder.getAbsolutePosition() * IntakeConstants.intakeDistalAbsoluteEncoderToRadians;
  }

  public double getIntakeProximalAbsoluteEncoderTicks(){
    return intakeProximalEncoder.getAbsolutePosition() * IntakeConstants.intakeProximalAbsoluteEncoderToTicks;
  }

  public double getIntakeDistalAbsoluteEncoderTicks(){
    return intakeDistalEncoder.getAbsolutePosition() * IntakeConstants.intakeDistalAbsoluteEncoderToTicks;
  }

  public double calculateIntakeProximalFeedforward(){
    return IntakeConstants.intakeMaxProximalGravityConstant * Math.cos(getAngleToCG());
  }

  public double calculateIntakeDistalFeedforward(){
    return IntakeConstants.intakeMaxDistalGravityConstant * Math.cos(getIntakeDistalPosition());
  }

  public double getAngleToCG(){
    double proximalCGX = Math.cos(getIntakeDistalPosition()) * IntakeConstants.intakeProximalCGDistance;
    double proximalCGY = Math.sin(getIntakeDistalPosition()) * IntakeConstants.intakeProximalCGDistance;
    double proximalLengthX = Math.cos(getIntakeDistalPosition()) * IntakeConstants.intakeProximalLength;
    double proximalLengthY = Math.sin(getIntakeDistalPosition()) * IntakeConstants.intakeProximalLength;

    double distalCGX = Math.cos(getIntakeDistalPosition()) * IntakeConstants.intakeDistalCGDistance + proximalLengthX;
    double distalCGY = Math.sin(getIntakeDistalPosition()) * IntakeConstants.intakeDistalCGDistance + proximalLengthY;

    double averageXG = ((IntakeConstants.intakeDistalMass * distalCGX) + IntakeConstants.intakeProximalMass * proximalCGX)
        / (IntakeConstants.intakeDistalMass + IntakeConstants.intakeProximalMass);
    double averageYG = ((IntakeConstants.intakeDistalMass * distalCGY) + IntakeConstants.intakeProximalMass * proximalCGY)
        / (IntakeConstants.intakeDistalMass + IntakeConstants.intakeProximalMass);

    return Math.atan(averageYG / averageXG);
  }

  public void setWheelSpeed(double stpt){
    intakeWheelMotor.set(ControlMode.PercentOutput, stpt);
  }

  public void stopWheels(){
    intakeWheelMotor.stopMotor();
  }

  public double getCurrentDraw(){
    return intakeWheelMotor.getStatorCurrent();
  }

  @Override
  public void periodic() {
    double proximalError = Math
        .abs(getIntakeProximalPosition() - stateHandler.getDesiredIntakePosition().getArmAngles().getProximalAngle());
    double distalError = Math.abs(getIntakeDistalPosition() - stateHandler.getDesiredIntakePosition().getArmAngles().getDistalAngle());

    stateHandler.updateIntakeInPosition(proximalError < IntakeConstants.errorThreshold && distalError < IntakeConstants.errorThreshold);
  }
}
