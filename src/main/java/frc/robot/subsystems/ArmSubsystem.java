// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  private WPI_TalonFX shoulderMotor = new WPI_TalonFX(ArmConstants.shoulderMotorID);
  private WPI_TalonFX elbowMotor = new WPI_TalonFX(ArmConstants.elbowMotorID);

  private ProfiledPIDController shoulderPIDController;
  private ProfiledPIDController elbowPIDController;

  private ArmFeedforward shoulderFeedForward;
  private ArmFeedforward elbowFeedForward;

  private double goalShoulderPosition;
  private double goalElbowPosition;
  
  public ArmSubsystem() {
    shoulderMotor.configFactoryDefault();
    elbowMotor.configFactoryDefault();
    
    shoulderMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.timeoutMs);
    elbowMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.timeoutMs);

    // shoulderMotor.config_kP(0, ArmConstants.shoulderkP, Constants.timeoutMs);
    // shoulderMotor.config_kI(0, ArmConstants.shoulderkI, Constants.timeoutMs);
    // shoulderMotor.config_kD(0, ArmConstants.shoulderkD, Constants.timeoutMs);

    // elbowMotor.config_kP(0, ArmConstants.elbowkP, Constants.timeoutMs);
    // elbowMotor.config_kI(0, ArmConstants.elbowkI, Constants.timeoutMs);
    // elbowMotor.config_kD(0, ArmConstants.elbowkD, Constants.timeoutMs);

    //WPILib PID controller :(
    shoulderPIDController = new ProfiledPIDController(ArmConstants.shoulderkP, ArmConstants.shoulderkI, ArmConstants.shoulderkD, 
      new TrapezoidProfile.Constraints(ArmConstants.shoulderkMaxVelocityRadPerSecond, ArmConstants.shoulderkAVoltSecondSquaredPerRad));

    elbowPIDController = new ProfiledPIDController(ArmConstants.elbowkP, ArmConstants.elbowkI, ArmConstants.elbowkD, 
      new TrapezoidProfile.Constraints(ArmConstants.elbowkMaxVelocityRadPerSecond, ArmConstants.elbowkMaxAccelerationRadPerSecSquared));

    //Arm feedforward stuff
    shoulderFeedForward = new ArmFeedforward(ArmConstants.shoulderkSVolts, ArmConstants.shoulderkGVolts, 
      ArmConstants.shoulderkVVoltSecondPerRad, ArmConstants.shoulderkAVoltSecondSquaredPerRad);

    elbowFeedForward = new ArmFeedforward(ArmConstants.elbowkSVolts, ArmConstants.elbowkGVolts, 
      ArmConstants.elbowkVVoltSecondPerRad, ArmConstants.elbowkAVoltSecondSquaredPerRad);

    setShoulderPosition(ArmConstants.kShoulderOffsetRads);
    setElbowPosition(ArmConstants.kElbowOffsetRads);

    goalShoulderPosition = ArmConstants.kShoulderOffsetRads;
    goalElbowPosition = ArmConstants.kElbowOffsetRads;

    shoulderMotor.setNeutralMode(NeutralMode.Brake);
    elbowMotor.setNeutralMode(NeutralMode.Brake);
  }

  public void setShoulderPosition(double setPoint){
    shoulderMotor.setSelectedSensorPosition(setPoint * ArmConstants.shoulderRadsToTicks);
  }

  public void setElbowPosition(double setPoint){
    elbowMotor.setSelectedSensorPosition(setPoint * ArmConstants.shoulderRadsToTicks);
  }

  public double getShoulderPosition(){
    return (shoulderMotor.getSelectedSensorPosition() * ArmConstants.shoulderTicksToRad) + ArmConstants.kShoulderOffsetRads;
  }

  public double getElbowPosition(){
    return (elbowMotor.getSelectedSensorPosition() * ArmConstants.elbowTicksToRad) + ArmConstants.kElbowOffsetRads;
  }

  public double getShoulderGoal(){
    return goalShoulderPosition;
  }

  public double getElbowGoal(){
    return goalElbowPosition;
  }

  public void setShoulderGoal(double sGoal){
    goalShoulderPosition = sGoal;
  }

  public void setElbowGoal(double eGoal){
    goalElbowPosition = eGoal;
  }

  public void setShoulderVoltage(double stpt){
    shoulderMotor.setVoltage(stpt);
  }

  public void setElbowVoltage(double stpt){
    elbowMotor.setVoltage(stpt);
  }

  public void stop(){
    shoulderMotor.stopMotor();
    elbowMotor.stopMotor();
  }

  public double getAngleToCG(){
    double shoulderCGX = Math.cos(getShoulderGoal()) * ArmConstants.shoulderCGDistance;
    double shoulderCGY = Math.sin(getShoulderGoal()) * ArmConstants.shoulderCGDistance;
    double shoulderLengthX = Math.cos(getShoulderGoal()) * ArmConstants.lengthOfShoulder;
    double shoulderLengthY = Math.sin(getShoulderGoal()) * ArmConstants.lengthOfShoulder;

    double elbowCGX = Math.cos(getElbowGoal()) * ArmConstants.elbowCGDistance + shoulderLengthX;
    double elbowCGY = Math.sin(getElbowGoal()) * ArmConstants.elbowCGDistance + shoulderLengthY;

    double averageXG = ((ArmConstants.elbowMass * elbowCGX) + ArmConstants.shoulderMass * shoulderCGX) / (ArmConstants.elbowMass + ArmConstants.shoulderMass);
    double averageYG = ((ArmConstants.elbowMass * elbowCGY) + ArmConstants.shoulderMass * shoulderCGY) / (ArmConstants.elbowMass + ArmConstants.shoulderMass);

    return Math.atan(averageYG / averageXG);
  }

  public double calculateShoulderFeedforward(){
    return shoulderFeedForward.calculate(getAngleToCG(), 0);
  }

  public double calculateElbowFeedforward(){
    return elbowFeedForward.calculate(getElbowGoal(), 0);
  }

  public double getShoulderPIDOutput(){
    return shoulderPIDController.calculate(getShoulderPosition(), goalShoulderPosition);
  }

  public double getElbowPIDOutput(){
    return elbowPIDController.calculate(getElbowPosition(), goalElbowPosition);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
