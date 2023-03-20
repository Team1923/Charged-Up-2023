// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FalconConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.util.StateHandler;
import frc.robot.util.StateVariables.IntakePositions;
import frc.robot.util.StateVariables.IntakeWheelSpeeds;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private WPI_TalonFX intakeArmMaster = new WPI_TalonFX(IntakeConstants.intakeArmMasterID, "rio");
  private WPI_TalonFX intakeArmFollower = new WPI_TalonFX(IntakeConstants.intakeArmFollowerID, "rio");
  private WPI_TalonFX leftIntakeWheelMotor = new WPI_TalonFX(IntakeConstants.leftIntakeWheelMotor, "rio");
  private WPI_TalonFX rightIntakeWheelMotor = new WPI_TalonFX(IntakeConstants.rightIntakeWheelMotor, "rio");

  private DutyCycleEncoder intakeEncoder = new DutyCycleEncoder(
      IntakeConstants.intakeDistalAbsoluteEncoderID);

  private DigitalInput gamePieceSensor = new DigitalInput(7);

  private StateHandler stateHandler = StateHandler.getInstance();

  public IntakeSubsystem() {
    intakeArmMaster.configFactoryDefault();
    intakeArmFollower.configFactoryDefault();
    leftIntakeWheelMotor.configFactoryDefault();
    rightIntakeWheelMotor.configFactoryDefault();

    intakeArmMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, FalconConstants.timeoutMs);
    intakeArmFollower.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, FalconConstants.timeoutMs);

    intakeArmFollower.follow(intakeArmMaster);
    intakeArmFollower.setInverted(InvertType.OpposeMaster);

    intakeArmMaster.config_kP(0, IntakeConstants.intakeArmkP, FalconConstants.timeoutMs);
    intakeArmMaster.config_kI(0, IntakeConstants.intakeArmkI, FalconConstants.timeoutMs);
    intakeArmMaster.config_kD(0, IntakeConstants.intakeArmkD, FalconConstants.timeoutMs);

    intakeArmFollower.config_kP(0, IntakeConstants.intakeArmkP, FalconConstants.timeoutMs);
    intakeArmFollower.config_kI(0, IntakeConstants.intakeArmkI, FalconConstants.timeoutMs);
    intakeArmFollower.config_kD(0, IntakeConstants.intakeArmkD, FalconConstants.timeoutMs);

    intakeArmMaster.configMotionCruiseVelocity(IntakeConstants.maxIntakeArmVel);
    intakeArmMaster.configMotionAcceleration(IntakeConstants.maxIntakeArmAccel);
    intakeArmFollower.configMotionCruiseVelocity(IntakeConstants.maxIntakeArmVel);
    intakeArmFollower.configMotionAcceleration(IntakeConstants.maxIntakeArmAccel);

    intakeArmMaster.setNeutralMode(NeutralMode.Coast);
    intakeArmFollower.setNeutralMode(NeutralMode.Coast);
    
    leftIntakeWheelMotor.setNeutralMode(NeutralMode.Brake);
    rightIntakeWheelMotor.setNeutralMode(NeutralMode.Brake);

    resetIntakePosition();

    leftIntakeWheelMotor.setInverted(InvertType.InvertMotorOutput);
    rightIntakeWheelMotor.setInverted(InvertType.InvertMotorOutput);

  }


  public void resetIntakePosition() {
    double setZeroPosition = (getIntakeAbsoluteEncoderRads() - IntakeConstants.intakeArmEncoderZero
        + IntakeConstants.intakeArmHardstop)
        * IntakeConstants.intakeArmRadsToTicks;
    intakeArmMaster.setSelectedSensorPosition(setZeroPosition);

    intakeArmFollower.setSelectedSensorPosition(setZeroPosition);
  }

  public void setIntakePosition(double distalAngle) {
    intakeArmMaster.set(ControlMode.MotionMagic, distalAngle * IntakeConstants.intakeArmRadsToTicks,
        DemandType.ArbitraryFeedForward, calculateIntakeFeedforward());
    intakeArmFollower.set(ControlMode.MotionMagic, distalAngle * IntakeConstants.intakeArmRadsToTicks,
        DemandType.ArbitraryFeedForward, calculateIntakeFeedforward());
  }

  public double getIntakeArmPosition() {
    return ((intakeArmMaster.getSelectedSensorPosition() * IntakeConstants.intakeArmTicksToRad)
        + IntakeConstants.kIntakeDistalOffsetRads);
  }

  public double getIntakeAbsoluteEncoderRads() {
    return (intakeEncoder.getAbsolutePosition() *
        IntakeConstants.intakeDistalAbsoluteEncoderToRadians);
  }

  public double getIntakeAbsoluteEncoderTicks() {
    return (intakeEncoder.getAbsolutePosition() *
        IntakeConstants.intakeArmAbsoluteEncoderToTicks);
  }

  public double calculateIntakeFeedforward() {
    return IntakeConstants.intakeArmMaxGravityConstant * Math.cos(getIntakeArmPosition());
  }


  public void setRawWheelSpeed(double stpt) {
    leftIntakeWheelMotor.set(stpt);
    rightIntakeWheelMotor.set(-stpt);
  }

  public void setRawWheelSpeed(double left, double right) {
    leftIntakeWheelMotor.set(left);
    rightIntakeWheelMotor.set(right);
  }

  public void stopWheels() {
    leftIntakeWheelMotor.stopMotor();
    rightIntakeWheelMotor.stopMotor();
  }

  public boolean getGamePieceSensor() {
    return !gamePieceSensor.get();
  }

  public void stopIntake() {
    intakeArmMaster.stopMotor();
    intakeArmFollower.stopMotor();
  }

  public void disableMotionMagic() {
    intakeArmMaster.set(ControlMode.Disabled, 0);
  }

  @Override
  public void periodic() {

    if(DriverStation.isDisabled()) {
      disableMotionMagic();
      stateHandler.setDesiredIntakeWheelSpeed(IntakeWheelSpeeds.GRIP);
      stopWheels();
    } else {
      setIntakePosition(stateHandler.getDesiredIntakePosition().getArmAngles().getAngle());
      if(getGamePieceSensor() && stateHandler.getDesiredIntakePosition() == IntakePositions.INTAKE) {
        setRawWheelSpeed(IntakeWheelSpeeds.GRIP.getIntakeWheelSpeed().getWheelSpeed());
      } else {
        setRawWheelSpeed(stateHandler.getDesiredIntakeWheelSpeed().getIntakeWheelSpeed().getWheelSpeed());
      }
    }

    stateHandler.setHasGamePiece(getGamePieceSensor());


    SmartDashboard.putNumber("INTAKE Arm POSITION RADS: ", getIntakeArmPosition());

    SmartDashboard.putNumber("INTAKE ABSOLUTE Encoder Rads", getIntakeAbsoluteEncoderRads());

    SmartDashboard.putNumber("FALCON INTAKE MOTOR ENCODER", getIntakeArmPosition());

  }
}
