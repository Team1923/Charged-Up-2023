// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.FalconConstants;
import frc.robot.util.StateHandler;
import frc.robot.util.StateVariables.ArmPositions;
import frc.robot.util.StateVariables.CurrentRobotDirection;
import frc.robot.util.StateVariables.GamePieceMode;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  private WPI_TalonFX proximalMotor = new WPI_TalonFX(ArmConstants.proximalMotorID);
  private WPI_TalonFX distalMotor = new WPI_TalonFX(ArmConstants.distalMotorID);
  private DutyCycleEncoder proximalEncoder = new DutyCycleEncoder(ArmConstants.proximalEncoderID); // change this
  private DutyCycleEncoder distalEncoder = new DutyCycleEncoder(ArmConstants.distalEncoderID);
  private StateHandler stateHandler = StateHandler.getInstance();

  
  

  public ArmSubsystem() {
    proximalMotor.configFactoryDefault();
    distalMotor.configFactoryDefault();

    proximalMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, FalconConstants.timeoutMs);
    distalMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, FalconConstants.timeoutMs);

    proximalMotor.config_kP(0, ArmConstants.proximalkP, FalconConstants.timeoutMs);
    proximalMotor.config_kI(0, ArmConstants.proximalkI, FalconConstants.timeoutMs);
    proximalMotor.config_kD(0, ArmConstants.proximalkD, FalconConstants.timeoutMs);

    distalMotor.config_kP(0, ArmConstants.distalkP, FalconConstants.timeoutMs);
    distalMotor.config_kI(0, ArmConstants.distalkI, FalconConstants.timeoutMs);
    distalMotor.config_kD(0, ArmConstants.distalkD, FalconConstants.timeoutMs);

    proximalMotor.configMotionCruiseVelocity(ArmConstants.maxProximalVel);
    proximalMotor.configMotionAcceleration(ArmConstants.maxProximalAccel);
    distalMotor.configMotionCruiseVelocity(ArmConstants.maxDistalVel);
    distalMotor.configMotionAcceleration(ArmConstants.maxDistalAccel);

    proximalMotor.setNeutralMode(NeutralMode.Brake);
    distalMotor.setNeutralMode(NeutralMode.Brake);

    resetDistalPosition();
    resetDistalPosition();

    
  }

  public void resetProximalPosition() {
    proximalMotor.setSelectedSensorPosition(
        (getProximalAbsoluteEncoderRads() - ArmConstants.proximalEncoderHardstop + ArmConstants.proximalHardstop)
            * ArmConstants.proximalRadsToTicks);
  }

  public void resetDistalPosition() {
    distalMotor.setSelectedSensorPosition(
        (getDistalAbsoluteEncoderRads() - ArmConstants.distalEncoderHardstop + ArmConstants.distalHardstop)
            * ArmConstants.distalRadsToTicks * ArmConstants.distalRadsToTicks);
  }

  // public void setproximalPosition(double setPoint){
  // proximalMotor.set(ControlMode.MotionMagic, setPoint,
  // DemandType.ArbitraryFeedForward, calculateproximalFeedforward());
  // }

  // public void setdistalPosition(double setPoint){
  // distalMotor.set(ControlMode.MotionMagic, -setPoint,
  // DemandType.ArbitraryFeedForward, calculatedistalFeedforward());
  // }

  public void setProximalPosition(double proximalAngle) {
    proximalMotor.set(ControlMode.MotionMagic, proximalAngle * ArmConstants.proximalRadsToTicks,
        DemandType.ArbitraryFeedForward, calculateProximalFeedforward());
  }

  public void setDistalPosition(double distalAngle) {
    distalMotor.set(ControlMode.MotionMagic, distalAngle * ArmConstants.distalRadsToTicks,
        DemandType.ArbitraryFeedForward, calculateDistalFeedforward());
  }

  public double[] calculateArmCartesian(double x, double y) {
    double distalGoal = (Math.acos(
        ((x * x) + (y * y) - (Math.pow(ArmConstants.lengthOfProximal, 2)) - (Math.pow(ArmConstants.lengthOfDistal, 2)))
            / (2 * ArmConstants.lengthOfProximal * ArmConstants.lengthOfDistal)));

    double proximalGoal = (Math.atan(y / x) - Math.atan((ArmConstants.lengthOfDistal * Math.sin(distalGoal))
        / (ArmConstants.lengthOfProximal + ArmConstants.lengthOfDistal * Math.cos(distalGoal))));

    double[] conv = { proximalGoal, distalGoal };

    return conv;
  }

  public double getProximalPosition() {
    return (proximalMotor.getSelectedSensorPosition() * ArmConstants.proximalTicksToRad)
        + ArmConstants.kProximalOffsetRads;
  }

  public double getDistalPosition() {
    return (distalMotor.getSelectedSensorPosition() * ArmConstants.distalTicksToRad) + ArmConstants.kDistalOffsetRads;
  }

  public void setProximalVoltage(double stpt) {
    proximalMotor.setVoltage(stpt);
  }

  public void setDistalVoltage(double stpt) {
    distalMotor.setVoltage(stpt);
  }

  public void stop() {
    proximalMotor.stopMotor();
    distalMotor.stopMotor();
  }

  public double getAngleToCG() {
    double proximalCGX = Math.cos(getDistalPosition()) * ArmConstants.proximalCGDistance;
    double proximalCGY = Math.sin(getDistalPosition()) * ArmConstants.proximalCGDistance;
    double proximalLengthX = Math.cos(getDistalPosition()) * ArmConstants.lengthOfProximal;
    double proximalLengthY = Math.sin(getDistalPosition()) * ArmConstants.lengthOfProximal;

    double distalCGX = Math.cos(getDistalPosition()) * ArmConstants.distalCGDistance + proximalLengthX;
    double distalCGY = Math.sin(getDistalPosition()) * ArmConstants.distalCGDistance + proximalLengthY;

    double averageXG = ((ArmConstants.distalMass * distalCGX) + ArmConstants.proximalMass * proximalCGX)
        / (ArmConstants.distalMass + ArmConstants.proximalMass);
    double averageYG = ((ArmConstants.distalMass * distalCGY) + ArmConstants.proximalMass * proximalCGY)
        / (ArmConstants.distalMass + ArmConstants.proximalMass);

    return Math.atan(averageYG / averageXG);
  }

  public double calculateProximalFeedforward() {
    return ArmConstants.maxProximalGravityConstant * Math.cos(getAngleToCG());
  }

  public double calculateDistalFeedforward() {
    return ArmConstants.maxDistalGravityConstant * Math.cos(getDistalPosition());
  }

  public double[] anglesToCartesian(double proximalTheta, double distalTheta) {

    double x = ArmConstants.lengthOfProximal * Math.cos(proximalTheta)
        + ArmConstants.lengthOfDistal * Math.cos(proximalTheta + distalTheta);
    double y = ArmConstants.lengthOfProximal * Math.sin(proximalTheta)
        + ArmConstants.lengthOfDistal * Math.sin(proximalTheta + distalTheta);

    double[] conv = { x, y };
    return conv;
  }

  public void setCoast() {
    distalMotor.setNeutralMode(NeutralMode.Coast);
    proximalMotor.setNeutralMode(NeutralMode.Coast);
  }

  public void setBrake() {
    distalMotor.setNeutralMode(NeutralMode.Brake);
    proximalMotor.setNeutralMode(NeutralMode.Brake);
  }

  public double getProximalAbsoluteEncoderRads() {
    return proximalEncoder.getAbsolutePosition() * ArmConstants.proximalAbsoluteEncoderToRadians;
  }

  public double getDistalAbsoluteEncoderRads() {
    return distalEncoder.getAbsolutePosition() * ArmConstants.distalAbsoluteEncoderToRadians;
  }

  public double getProximalAbsoluteEncoderTicks() {
    return proximalEncoder.getAbsolutePosition() * ArmConstants.proximalAbsoluteEncoderToTicks;
  }

  public double getDistalAbsoluteEncoderTicks() {
    return distalEncoder.getAbsolutePosition() * ArmConstants.distalAbsoluteEncoderToTicks;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("distal Position:", Math.toDegrees(getDistalPosition()));
    SmartDashboard.putNumber("proximal Position:", Math.toDegrees(getDistalPosition()));

    SmartDashboard.putNumber("Proximal Rads", getProximalAbsoluteEncoderRads());
    SmartDashboard.putNumber("Distal Rads", getDistalAbsoluteEncoderRads());

    double proximalError = Math
        .abs(getProximalPosition() - stateHandler.getArmDesiredPosition().getArmAngles().getProximalAngle());
    double distalError = Math
        .abs(getDistalPosition() - stateHandler.getArmDesiredPosition().getArmAngles().getDistalAngle());

    boolean withinThreshold = proximalError < ArmConstants.errorThreshold && distalError < ArmConstants.errorThreshold;

    stateHandler.setArmInPosition(withinThreshold);

    if (withinThreshold) {
      stateHandler.setCurrentArmPosition(stateHandler.getArmDesiredPosition());
    }

    SmartDashboard.putString("DESIRED ARM State", stateHandler.getArmDesiredPosition().toString());
    SmartDashboard.putString("CURRENT ARM State", stateHandler.getCurrentArmPosition().toString());

    double currentArmPosition = SmartDashboard.getNumber("INPUT ARM POSITION", 0);
    SmartDashboard.putNumber("INPUT ARM POSITION", currentArmPosition);
    if(currentArmPosition == 0){
      stateHandler.setArmDesiredState(ArmPositions.STOW);
    } else if (currentArmPosition == 1){
      stateHandler.setArmDesiredState(ArmPositions.COBRA);
    } else if (currentArmPosition == 2){
      stateHandler.setArmDesiredState(ArmPositions.CLEAR);
    } else if(currentArmPosition == 3){
      stateHandler.setArmDesiredState(ArmPositions.CONE_HIGH);
    } else if(currentArmPosition == 4){
      stateHandler.setArmDesiredState(ArmPositions.CONE_MID);
    } else if(currentArmPosition == 5){
      stateHandler.setArmDesiredState(ArmPositions.CUBE_HIGH);
    } else if(currentArmPosition == 6){
      stateHandler.setArmDesiredState(ArmPositions.CUBE_MID);
    } else if(currentArmPosition == 7){
      stateHandler.setArmDesiredState(ArmPositions.LOW);
    }

    double currentGamePiece = SmartDashboard.getNumber("INPUT GAME PIECE", 0);
    SmartDashboard.putNumber("INPUT GAME PIECE", currentArmPosition);
    if(currentGamePiece == 0){
      stateHandler.setGamePieceMode(GamePieceMode.CONE);
    } else{
      stateHandler.setGamePieceMode(GamePieceMode.CUBE);
    }

    double currentDirection = SmartDashboard.getNumber("INPUT CURRENT DIRECTION", 0);
    SmartDashboard.putNumber("INPUT CURRENT DIRECTION", currentDirection);
    if(currentDirection == 0) {
      stateHandler.setRobotDirection(CurrentRobotDirection.LEFT);
    } else{
      stateHandler.setRobotDirection(CurrentRobotDirection.RIGHT);
    }
    
    
    

  }
}
