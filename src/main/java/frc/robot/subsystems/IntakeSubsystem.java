// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FalconConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.EmergencyCommands.EStopIntakeCommand;
import frc.robot.util.StateHandler;
import frc.robot.util.StateVariables.GamePieceMode;
import frc.robot.util.math.RollingAvgDouble;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private WPI_TalonFX intakeProximalMotor = new WPI_TalonFX(IntakeConstants.intakeProximalID, "rio");
  private WPI_TalonFX intakeDistalMotor = new WPI_TalonFX(IntakeConstants.intakeDistalID);
  private WPI_TalonFX leftIntakeWheelMotor = new WPI_TalonFX(IntakeConstants.leftIntakeWheelMotor, "rio");
  private WPI_TalonFX rightIntakeWheelMotor = new WPI_TalonFX(IntakeConstants.rightIntakeWheelMotor, "rio");

  private DoubleSolenoid intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);

  private DutyCycleEncoder intakeProximalEncoder = new DutyCycleEncoder(
      IntakeConstants.intakeProximalAbsoluteEncoderID);
  private DutyCycleEncoder intakeDistalEncoder = new DutyCycleEncoder(IntakeConstants.intakeDistalAbsoluteEncoderID);

  private StateHandler stateHandler = StateHandler.getInstance();

  private RollingAvgDouble averageCurrentDraw = new RollingAvgDouble(50);

  public IntakeSubsystem() {
    intakeProximalMotor.configFactoryDefault();
    intakeDistalMotor.configFactoryDefault();
    leftIntakeWheelMotor.configFactoryDefault();
    rightIntakeWheelMotor.configFactoryDefault();

    intakeProximalMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, FalconConstants.timeoutMs);
    intakeDistalMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, FalconConstants.timeoutMs);

    intakeProximalMotor.config_kP(0, IntakeConstants.intakeProximalkP, FalconConstants.timeoutMs);
    intakeProximalMotor.config_kI(0, IntakeConstants.intakeProximalkI, FalconConstants.timeoutMs);
    intakeProximalMotor.config_kD(0, IntakeConstants.intakeProximalkD, FalconConstants.timeoutMs);

    intakeDistalMotor.config_kP(0, IntakeConstants.intakeDistalkP, FalconConstants.timeoutMs);
    intakeDistalMotor.config_kI(0, IntakeConstants.intakeDistalkI, FalconConstants.timeoutMs);
    intakeDistalMotor.config_kD(0, IntakeConstants.intakeDistalkD, FalconConstants.timeoutMs);

    intakeProximalMotor.configMotionCruiseVelocity(IntakeConstants.maxIntakeProximalVel);
    intakeProximalMotor.configMotionAcceleration(IntakeConstants.maxIntakeProximalAccel);
    intakeDistalMotor.configMotionCruiseVelocity(IntakeConstants.maxIntakeDistalVel);
    intakeDistalMotor.configMotionAcceleration(IntakeConstants.maxIntakeDistalAccel);

    intakeProximalMotor.setNeutralMode(NeutralMode.Coast);
    intakeDistalMotor.setNeutralMode(NeutralMode.Coast);
    leftIntakeWheelMotor.setNeutralMode(NeutralMode.Brake);
    rightIntakeWheelMotor.setNeutralMode(NeutralMode.Brake);

    resetIntakeProximalPosition();
    resetIntakeDistalPosition();

    //intakeProximalMotor.setSelectedSensorPosition(0);
    //intakeDistalMotor.setSelectedSensorPosition(0);
  }

  // public void resetIntakeProximalPosition(double angle) {
  //   intakeProximalMotor.setSelectedSensorPosition(angle * IntakeConstants.intakeProximalRadsToTicks);
  // }

  // public void resetIntakeDistalPosition(double angle) {
  //   intakeDistalMotor.setSelectedSensorPosition(angle * IntakeConstants.intakeDistalRadsToTicks);
  // }

  //For both the Proximal and Distal, using the Encoders and physical hardstops, we can reset their position
  public void resetIntakeProximalPosition() {
    intakeProximalMotor.setSelectedSensorPosition(
        (getIntakeProximalAbsoluteEncoderRads() - IntakeConstants.proximalEncoderZero + IntakeConstants.intakeProximalHardstop)
            * IntakeConstants.intakeProximalRadsToTicks);
    //proximalMotor.setSelectedSensorPosition(ArmPositions.STOW.getArmAngles().getProximalAngle());
  }

  public void resetIntakeDistalPosition() {
    intakeDistalMotor.setSelectedSensorPosition(
        (getIntakeDistalAbsoluteEncoderRads() - IntakeConstants.distalEncoderZero + IntakeConstants.intakeDistalHardstop)
            * IntakeConstants.intakeDistalRadsToTicks);
    //proximalMotor.setSelectedSensorPosition(ArmPositions.STOW.getArmAngles().getProximalAngle());
  }

  public void setIntakeProximalPosition(double proximalAngle) {
    intakeProximalMotor.set(ControlMode.MotionMagic, proximalAngle * IntakeConstants.intakeProximalRadsToTicks,
        DemandType.ArbitraryFeedForward, calculateIntakeProximalFeedforward());
  }

  public void setIntakeDistalPosition(double distalAngle) {
    intakeDistalMotor.set(ControlMode.MotionMagic, distalAngle * IntakeConstants.intakeDistalRadsToTicks,
        DemandType.ArbitraryFeedForward, calculateIntakeDistalFeedforward());
  }

  public double getIntakeProximalPosition() {
    return (intakeProximalMotor.getSelectedSensorPosition() * IntakeConstants.intakeProximalTicksToRad)
        + IntakeConstants.kIntakeProximalOffsetRads;
  }

  public double getIntakeDistalPosition() {
    return (intakeDistalMotor.getSelectedSensorPosition() * IntakeConstants.intakeDistalTicksToRad)
        + IntakeConstants.kIntakeDistalOffsetRads;
  }

  public double getIntakeProximalAbsoluteEncoderRads() {
    return 2*Math.PI -  (intakeProximalEncoder.getAbsolutePosition() *
        IntakeConstants.intakeProximalAbsoluteEncoderToRadians);
  }

  public double getIntakeDistalAbsoluteEncoderRads() {
    return intakeDistalEncoder.getAbsolutePosition() *
        IntakeConstants.intakeDistalAbsoluteEncoderToRadians;
  }

  public double getIntakeProximalAbsoluteEncoderTicks() {
    return (intakeProximalEncoder.getAbsolutePosition() *
        IntakeConstants.intakeProximalAbsoluteEncoderToTicks);
  }

  public double getIntakeDistalAbsoluteEncoderTicks() {
    return intakeDistalEncoder.getAbsolutePosition() *
        IntakeConstants.intakeDistalAbsoluteEncoderToTicks;
  }

  public double calculateIntakeProximalFeedforward() {
    return IntakeConstants.intakeMaxProximalGravityConstant * Math.cos(getAngleToCG());
  }

  public double calculateIntakeDistalFeedforward() {
    return IntakeConstants.intakeMaxDistalGravityConstant * Math.cos(getIntakeDistalPosition());
  }

  //Using measured constants, we are able to find the angle from the C.O.M, with respect to the horizontal
  public double getAngleToCG() {
    double proximalCGX = Math.cos(getIntakeDistalPosition()) * IntakeConstants.intakeProximalCGDistance;
    double proximalCGY = Math.sin(getIntakeDistalPosition()) * IntakeConstants.intakeProximalCGDistance;
    double proximalLengthX = Math.cos(getIntakeDistalPosition()) * IntakeConstants.intakeProximalLength;
    double proximalLengthY = Math.sin(getIntakeDistalPosition()) * IntakeConstants.intakeProximalLength;

    double distalCGX = Math.cos(getIntakeDistalPosition()) * IntakeConstants.intakeDistalCGDistance + proximalLengthX;
    double distalCGY = Math.sin(getIntakeDistalPosition()) * IntakeConstants.intakeDistalCGDistance + proximalLengthY;

    double averageXG = ((IntakeConstants.intakeDistalMass * distalCGX)
        + IntakeConstants.intakeProximalMass * proximalCGX)
        / (IntakeConstants.intakeDistalMass + IntakeConstants.intakeProximalMass);
    double averageYG = ((IntakeConstants.intakeDistalMass * distalCGY)
        + IntakeConstants.intakeProximalMass * proximalCGY)
        / (IntakeConstants.intakeDistalMass + IntakeConstants.intakeProximalMass);

    return Math.atan(averageYG / averageXG);
  }

  public void setWheelSpeed() {
    if (stateHandler.getGamePieceMode() == GamePieceMode.CUBE) {
      leftIntakeWheelMotor.set(ControlMode.PercentOutput, IntakeConstants.cubeIntakeSpeed);
      rightIntakeWheelMotor.set(ControlMode.PercentOutput, -IntakeConstants.cubeIntakeSpeed);
    } else {
      leftIntakeWheelMotor.set(ControlMode.PercentOutput, IntakeConstants.coneIntakeSpeed);
      rightIntakeWheelMotor.set(ControlMode.PercentOutput, IntakeConstants.coneIntakeSpeed);
    }
  }

  public void setWheelSpeed(double stpt) {
    if (stateHandler.getGamePieceMode() == GamePieceMode.CUBE) {
      leftIntakeWheelMotor.set(ControlMode.PercentOutput, stpt);
      rightIntakeWheelMotor.set(ControlMode.PercentOutput, -stpt);
    } else {
      leftIntakeWheelMotor.set(ControlMode.PercentOutput, stpt);
      rightIntakeWheelMotor.set(ControlMode.PercentOutput, stpt);
    }
  }

  public void setRawWheelSpeed(double stpt) {
    leftIntakeWheelMotor.set(stpt);
    rightIntakeWheelMotor.set(-stpt);
  }

  public void stopWheels() {
    leftIntakeWheelMotor.stopMotor();
  }

  public double getCurrentDraw() {
    return leftIntakeWheelMotor.getStatorCurrent();
  }
//Gets the rolling average, or current over period of time, and can be used to monitor current spikes
  public void updateCurrentRollingAvg() {
    averageCurrentDraw.add(getCurrentDraw());
  }
  
  public boolean getAverageCurrentAboveThreshold(double goal) {
    return averageCurrentDraw.getAvg() > goal;
  }

  public void setSolenoid(boolean output) {
    intakeSolenoid.set(output ? Value.kForward : Value.kReverse);
  }

  public boolean getSolenoid() {
    return intakeSolenoid.get() == Value.kForward ? true : false;
  }
//By checking the raw current, we know that we have a game piece if it spikes above a threshold value
  public boolean intakeHasGamePiece() {
    return ((stateHandler.getGamePieceMode() == GamePieceMode.CONE)
        && (getCurrentDraw() > IntakeConstants.coneCurrentThreshold))
        || ((stateHandler.getGamePieceMode() == GamePieceMode.CUBE)
            && (getCurrentDraw() > IntakeConstants.cubeCurrentThreshold));
  }

  public double getIntakeProximalCurrent(){
    return intakeProximalMotor.getStatorCurrent();
  }

  public double getIntakeDistalCurrent(){
    return intakeDistalMotor.getStatorCurrent();
  }

  public void stopIntake(){
    intakeProximalMotor.stopMotor();
    intakeDistalMotor.stopMotor();
  }

  @Override
  public void periodic() {
    //If the current for either the proximal or distal is above 50, we stop the motors and stop running the intake subsystem
    if(getIntakeProximalCurrent() > 50 || getIntakeDistalCurrent() > 50){
      intakeProximalMotor.stopMotor();
      intakeDistalMotor.stopMotor();
      CommandScheduler.getInstance().schedule(new EStopIntakeCommand(this));
    }

    double proximalError = Math
        .abs(getIntakeProximalPosition() - stateHandler.getDesiredIntakePosition().getArmAngles().getProximalAngle());
    double distalError = Math
        .abs(getIntakeDistalPosition() - stateHandler.getDesiredIntakePosition().getArmAngles().getDistalAngle());

    boolean withinThreshold = proximalError < IntakeConstants.errorThreshold
        && distalError < IntakeConstants.errorThreshold;

    stateHandler.setIntakeInPosition(withinThreshold);
   
    if (withinThreshold) {
      stateHandler.setCurrentIntakePosition(stateHandler.getDesiredIntakePosition());
    }

  }
}
