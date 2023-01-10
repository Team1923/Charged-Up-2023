// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
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

  private double goalShoulderPosition;
  private double goalElbowPosition;
  
  public ArmSubsystem() {
    shoulderMotor.configFactoryDefault();
    elbowMotor.configFactoryDefault();
    
    shoulderMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.timeoutMs);
    elbowMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.timeoutMs);

    shoulderMotor.config_kP(0, ArmConstants.shoulderkP, Constants.timeoutMs);
    shoulderMotor.config_kI(0, ArmConstants.shoulderkI, Constants.timeoutMs);
    shoulderMotor.config_kD(0, ArmConstants.shoulderkD, Constants.timeoutMs);

    elbowMotor.config_kP(0, ArmConstants.elbowkP, Constants.timeoutMs);
    elbowMotor.config_kI(0, ArmConstants.elbowkI, Constants.timeoutMs);
    elbowMotor.config_kD(0, ArmConstants.elbowkD, Constants.timeoutMs);


    resetShoulderPosition(0);
    resetElbowPosition(0);

    goalShoulderPosition = ArmConstants.kShoulderOffsetRads;
    goalElbowPosition = ArmConstants.kElbowOffsetRads;

    shoulderMotor.configMotionCruiseVelocity(ArmConstants.maxShoulderVel);
    shoulderMotor.configMotionAcceleration(ArmConstants.maxShoulderAccel);
    elbowMotor.configMotionCruiseVelocity(ArmConstants.maxElbowVel);
    elbowMotor.configMotionAcceleration(ArmConstants.maxElbowAccel);

    shoulderMotor.setNeutralMode(NeutralMode.Brake);
    elbowMotor.setNeutralMode(NeutralMode.Brake);
  }

  public void resetShoulderPosition(double setPoint){
    shoulderMotor.setSelectedSensorPosition(setPoint * ArmConstants.shoulderRadsToTicks);
  }

  public void resetElbowPosition(double setPoint){
    elbowMotor.setSelectedSensorPosition(setPoint * ArmConstants.shoulderRadsToTicks);
  }

  public void setShoulderPosition(double setPoint){
    shoulderMotor.set(ControlMode.MotionMagic, setPoint, DemandType.ArbitraryFeedForward, calculateShoulderFeedforward());
  }

  public void setElbowPosition(double setPoint){
    elbowMotor.set(ControlMode.MotionMagic, setPoint, DemandType.ArbitraryFeedForward, calculateElbowFeedforward());
  }

  public void setShoulderPosition(){
    shoulderMotor.set(ControlMode.MotionMagic, getShoulderGoal() * ArmConstants.shoulderRadsToTicks, DemandType.ArbitraryFeedForward, calculateShoulderFeedforward());
  }

  public void setElbowPosition(){
    elbowMotor.set(ControlMode.MotionMagic, (getElbowGoal() * ArmConstants.elbowRadsToTicks), DemandType.ArbitraryFeedForward, calculateElbowFeedforward());
  }

  public void setArmCartesian(double x, double y){
    double elbowGoal = (Math.acos(((x*x) + (y*y) - (Math.pow(ArmConstants.lengthOfShoulder, 2)) - (Math.pow(ArmConstants.lengthOfElbow, 2))) 
    / (2 * ArmConstants.lengthOfShoulder * ArmConstants.lengthOfElbow)));

    double shoulderGoal = (Math.atan(y/x) - 
      Math.atan((ArmConstants.lengthOfElbow * Math.sin(elbowGoal)) / (ArmConstants.lengthOfShoulder + ArmConstants.lengthOfElbow * Math.cos(elbowGoal))));

    setShoulderGoal(shoulderGoal);
    setElbowGoal(elbowGoal);
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
    double shoulderCGX = Math.cos(getShoulderPosition()) * ArmConstants.shoulderCGDistance;
    double shoulderCGY = Math.sin(getShoulderPosition()) * ArmConstants.shoulderCGDistance;
    double shoulderLengthX = Math.cos(getShoulderPosition()) * ArmConstants.lengthOfShoulder;
    double shoulderLengthY = Math.sin(getShoulderPosition()) * ArmConstants.lengthOfShoulder;

    double elbowCGX = Math.cos(getElbowPosition()) * ArmConstants.elbowCGDistance + shoulderLengthX;
    double elbowCGY = Math.sin(getElbowPosition()) * ArmConstants.elbowCGDistance + shoulderLengthY;

    double averageXG = ((ArmConstants.elbowMass * elbowCGX) + ArmConstants.shoulderMass * shoulderCGX) / (ArmConstants.elbowMass + ArmConstants.shoulderMass);
    double averageYG = ((ArmConstants.elbowMass * elbowCGY) + ArmConstants.shoulderMass * shoulderCGY) / (ArmConstants.elbowMass + ArmConstants.shoulderMass);

    return Math.atan(averageYG / averageXG);
  }

  public double calculateShoulderFeedforward(){
    return ArmConstants.maxShoulderGravityConstant * Math.cos(getAngleToCG());
  }

  public double calculateElbowFeedforward(){
    return ArmConstants.maxElbowGravityConstant * Math.cos(getElbowPosition());
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
