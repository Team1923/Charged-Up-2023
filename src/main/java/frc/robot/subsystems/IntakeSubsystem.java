// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
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

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private WPI_TalonFX intakeProximalMotor = new WPI_TalonFX(IntakeConstants.intakeProximalID, "rio");
  private WPI_TalonFX intakeDistalMotor = new WPI_TalonFX(IntakeConstants.intakeDistalID, "rio");
  private WPI_TalonFX leftIntakeWheelMotor = new WPI_TalonFX(IntakeConstants.leftIntakeWheelMotor, "rio");
  private WPI_TalonFX rightIntakeWheelMotor = new WPI_TalonFX(IntakeConstants.rightIntakeWheelMotor, "rio");

  private DoubleSolenoid intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);

  private DutyCycleEncoder intakeProximalEncoder = new DutyCycleEncoder(
      IntakeConstants.intakeProximalAbsoluteEncoderID);
  private DutyCycleEncoder intakeDistalEncoder = new DutyCycleEncoder(IntakeConstants.intakeDistalAbsoluteEncoderID);

  private DigitalInput gamePieceSensor = new DigitalInput(9);

  private StateHandler stateHandler = StateHandler.getInstance();


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

  }


  //For both the Proximal and Distal, using the Encoders and physical hardstops, we can reset their position
  public void resetIntakeProximalPosition() {
    intakeProximalMotor.setSelectedSensorPosition(
        (getIntakeProximalAbsoluteEncoderRads() - IntakeConstants.proximalEncoderZero + IntakeConstants.intakeProximalHardstop)
            * IntakeConstants.intakeProximalRadsToTicks);
  }

  public void resetIntakeDistalPosition() {
    intakeDistalMotor.setSelectedSensorPosition(
        (getIntakeDistalAbsoluteEncoderRads() - IntakeConstants.distalEncoderZero + IntakeConstants.intakeDistalHardstop)
            * IntakeConstants.intakeDistalRadsToTicks);
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

  public void setRawWheelSpeed(double left, double right) {
    leftIntakeWheelMotor.set(left);
    rightIntakeWheelMotor.set(right);
  }

  public void stopWheels() {
    leftIntakeWheelMotor.stopMotor();
  }

  public double getCurrentDraw() {
    return leftIntakeWheelMotor.getStatorCurrent();
  }


  public void setSolenoid(boolean output) {
    intakeSolenoid.set(output ? Value.kForward : Value.kReverse);
  }

  public boolean getSolenoid() {
    return intakeSolenoid.get() == Value.kForward ? true : false;
  }

  public double getIntakeProximalCurrent(){
    return intakeProximalMotor.getStatorCurrent();
  }

  public double getIntakeDistalCurrent(){
    return intakeDistalMotor.getStatorCurrent();
  }

  public boolean getGamePieceSensor() {
    return !gamePieceSensor.get();
  }

  public void stopIntake(){
    intakeProximalMotor.stopMotor();
    intakeDistalMotor.stopMotor();
  }

  public void disableMotionMagic() {
    intakeProximalMotor.set(ControlMode.Disabled, 0);
    intakeDistalMotor.set(ControlMode.Disabled, 0);
  }

  @Override
  public void periodic() {

    // if(DriverStation.isDisabled()) {
    //   disableMotionMagic();
    // }

    //If the current for either the proximal or distal is above 50, we stop the motors and stop running the intake subsystem
    if(getIntakeProximalCurrent() > 100 || getIntakeDistalCurrent() > 100){
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
   
    if (withinThreshold) {
      stateHandler.setCurrentIntakePosition(stateHandler.getDesiredIntakePosition());
    }


    // SmartDashboard.putString("DESIRED INTAKE State", stateHandler.getDesiredIntakePosition().toString());
    // SmartDashboard.putString("CURRENT INTAKE State", stateHandler.getCurrentIntakePosition().toString());

    SmartDashboard.putNumber("INTAKE PROXIMAL POSITION RADS: ", getIntakeProximalPosition());
    SmartDashboard.putNumber("INTAKE DISTAL POSITION RADS: ", getIntakeDistalPosition());

    SmartDashboard.putNumber("INTAKE ABSOLUTE Proximal Encoder Rads", getIntakeProximalAbsoluteEncoderRads());
    SmartDashboard.putNumber("INTAKE ABSOLUTE Distal Encoder Rads", getIntakeDistalAbsoluteEncoderRads());
    
    SmartDashboard.putString("Desired Intake Position", stateHandler.getDesiredIntakePosition().toString());

    SmartDashboard.putBoolean("LIMIT SWITCH", getGamePieceSensor());

    // SmartDashboard.putString("Scoring Location Vertical", stateHandler.getCurrentVerticalLocation().toString());
    // SmartDashboard.putString("Scoring Location Horizontal", stateHandler.getCurrentHorizontalLocation().toString());

    // SmartDashboard.putNumber("Proixmal Closed Loop Error", intakeProximalMotor.getSelectedSensorPosition() - intakeProximalMotor.getClosedLoopTarget());
    // SmartDashboard.putNumber("Distal Closed Loop Error", intakeDistalMotor.getSelectedSensorPosition() - intakeDistalMotor.getClosedLoopTarget());

  }
}
