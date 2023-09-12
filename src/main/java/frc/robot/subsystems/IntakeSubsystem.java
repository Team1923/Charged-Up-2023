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
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FalconConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.IntakeCommands.EmergencyResetCommand;
import frc.robot.util.StateHandler;
import frc.robot.util.StateVariables.IntakePositions;
import frc.robot.util.StateVariables.IntakeWheelSpeeds;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private WPI_TalonFX intakeArmMaster = new WPI_TalonFX(IntakeConstants.intakeArmMasterID, "rio");
  private WPI_TalonFX intakeArmFollower = new WPI_TalonFX(IntakeConstants.intakeArmFollowerID, "rio");
  private WPI_TalonFX leftIntakeWheelMotor = new WPI_TalonFX(IntakeConstants.leftIntakeWheelMotor, "rio");
  private WPI_TalonFX rightIntakeWheelMotor = new WPI_TalonFX(IntakeConstants.rightIntakeWheelMotor, "rio");

  private WPI_TalonFX horizontalRollerMotor = new WPI_TalonFX(IntakeConstants.horizontalRollerID, "rio");

  private DoubleSolenoid rollerSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
  private DoubleSolenoid hardstopSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
  private DoubleSolenoid stickSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4, 5);

  private DutyCycleEncoder intakeEncoder = new DutyCycleEncoder(
      IntakeConstants.intakeAbosluteEncoderID);

  private DigitalInput gamePieceSensor = new DigitalInput(9);

  private StateHandler stateHandler = StateHandler.getInstance();

  private Timer hardstopChangeTimer;
  private Timer plopTimer;
  private Value hardstopValue = Value.kForward;

  public IntakeSubsystem() {

    hardstopChangeTimer = new Timer();
    plopTimer = new Timer();

    hardstopChangeTimer.start();

    intakeArmMaster.configFactoryDefault();
    intakeArmFollower.configFactoryDefault();
    leftIntakeWheelMotor.configFactoryDefault();
    rightIntakeWheelMotor.configFactoryDefault();
    horizontalRollerMotor.configFactoryDefault();

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

    intakeArmMaster.setNeutralMode(NeutralMode.Brake);
    intakeArmFollower.setNeutralMode(NeutralMode.Brake);

    leftIntakeWheelMotor.setNeutralMode(NeutralMode.Brake);
    rightIntakeWheelMotor.setNeutralMode(NeutralMode.Brake);
    horizontalRollerMotor.setNeutralMode(NeutralMode.Brake);

    Timer.delay(3);
    resetIntakePosition();

    leftIntakeWheelMotor.setInverted(InvertType.InvertMotorOutput);
    rightIntakeWheelMotor.setInverted(InvertType.InvertMotorOutput);
    horizontalRollerMotor.setInverted(InvertType.InvertMotorOutput);

  }

  public void resetIntakePosition() {
    double setZeroPosition = (getIntakeAbsoluteEncoderRads() - IntakeConstants.intakeArmEncoderZero
        + IntakeConstants.intakeArmHardstop)
        * IntakeConstants.intakeArmRadsToTicks;
    intakeArmMaster.setSelectedSensorPosition(setZeroPosition);
  }

  public void emergencyResetIntakePosition() {
    intakeArmMaster.setSelectedSensorPosition(
        (IntakePositions.SHOOT_TALL.getMainAngle().getAngle() + Math.toRadians(12)) * IntakeConstants.intakeArmRadsToTicks);
  }

  public void setIntakePosition(double distalAngle) {
    intakeArmMaster.set(ControlMode.MotionMagic, distalAngle * IntakeConstants.intakeArmRadsToTicks,
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

  public void setMainRawWheelSpeed(IntakeWheelSpeeds wheelSpeed) {
    leftIntakeWheelMotor.set(wheelSpeed.getIntakeWheelSpeed().getLeftSpeed());
    rightIntakeWheelMotor.set(-wheelSpeed.getIntakeWheelSpeed().getRightSpeed());
  }

  public void setHorizontalRawWheelSpeed(IntakeWheelSpeeds rollerSpeed) {
    horizontalRollerMotor.set(rollerSpeed.getIntakeWheelSpeed().getHorizontalRollerSpd());
  }

  public void setRollerSolenoid() {
    rollerSolenoid.set(stateHandler.getDesiredIntakePosition().getHorizontalSolenoid());
  }

  public void setHardstopSolenoid() {
    hardstopSolenoid.set(stateHandler.getDesiredIntakePosition().getHardstopSolenoid());
  }

  public void setStickSolenoid() {
    stickSolenoid.set(stateHandler.getStickOutSolenoid());
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

  public void setIntakeArmCoast() {
    intakeArmFollower.setNeutralMode(NeutralMode.Coast);
    intakeArmMaster.setNeutralMode(NeutralMode.Coast);
  }

  public Value getHardstopSolenoidOutput() {
    return hardstopSolenoid.get();
  }

  public void setRawIntakeArmSpeed(double stpt) {
    intakeArmMaster.set(stpt);
  }

  public double getRawIntakeArmCurrent() {
    return intakeArmMaster.getStatorCurrent();
  }


  @Override
  public void periodic() {

    stateHandler.setHasGamePiece(getGamePieceSensor());

    SmartDashboard.putNumber("INTAKE Arm POSITION RADS: ", getIntakeArmPosition());
    SmartDashboard.putNumber("INTAKE ABSOLUTE Encoder Rads", getIntakeAbsoluteEncoderRads());

    SmartDashboard.putString("Intake Speed", stateHandler.getDesiredIntakeWheelSpeed().toString());

    SmartDashboard.putString("INTAKE POSITION", stateHandler.getDesiredIntakePosition().toString());
    // SmartDashboard.putNumber("FALCON INTAKE MOTOR ENCODER", getIntakeArmPosition());

    // SmartDashboard.putString("Desired Intake Position", stateHandler.getDesiredIntakePosition().toString());
    // SmartDashboard.putString("Desired Intake Speeds", stateHandler.getDesiredIntakePosition().toString());

    // SmartDashboard.putNumber("Closed Loop error", intakeArmMaster.getClosedLoopError());

    SmartDashboard.putNumber("LEFT WHEEL Percent Out", leftIntakeWheelMotor.getMotorOutputPercent());

    SmartDashboard.putNumber("RIGHT WHEEL Percent Out", rightIntakeWheelMotor.getMotorOutputPercent());

    // SmartDashboard.putNumber("Intake Arm Velocity", intakeArmMaster.getSelectedSensorVelocity());

    // Runs when disabled
    if (DriverStation.isDisabled()) {
      disableMotionMagic();
      stateHandler.setDesiredIntakeWheelSpeed(IntakeWheelSpeeds.GRIP);
      stopWheels();
      return;
    }

    // if(stateHandler.getDesiredIntakePosition() == IntakePositions.INTAKE && getGamePieceSensor()) {
    //   stateHandler.setDesiredIntakePosition(IntakePositions.INTAKE_BAR_UP);
    // }

    // Update the intake solenoid based on the desired state
    setRollerSolenoid();

    // Set the hardstop solenoid to the desired value
    setHardstopSolenoid();

    // Set the stick Solenoid to the desired value
    setStickSolenoid();

    // Reset the timer for the hardstop solenoid if the value has changed since the
    // last loop
    if (hardstopValue != getHardstopSolenoidOutput()) {
      hardstopValue = getHardstopSolenoidOutput();
      hardstopChangeTimer.reset();
      hardstopChangeTimer.start();
    }

    if (!CommandScheduler.getInstance().isScheduled(EmergencyResetCommand.getInstance(this))) {
      // Set the intake to the desired setpoint
      if (hardstopChangeTimer.get() < 0.5) {
        setIntakePosition(stateHandler.getDesiredIntakePosition().getTempAngle().getAngle());
      } else {
        setIntakePosition(stateHandler.getDesiredIntakePosition().getMainAngle().getAngle());
      }
    }

    // Set the wheel speeds for the intake motors. If in intake and the sensor is
    // blocked, then run the wheel speeds at grip speeds.
    if ((stateHandler.getDesiredIntakePosition() == IntakePositions.INTAKE 
      || stateHandler.getDesiredIntakePosition() == IntakePositions.INTAKE_HIGHER)
        && (stateHandler.getDesiredIntakeWheelSpeed() == IntakeWheelSpeeds.SHOOT_LOW
            || stateHandler.getDesiredIntakeWheelSpeed() == IntakeWheelSpeeds.SHOOT_MID
            || stateHandler.getDesiredIntakeWheelSpeed() == IntakeWheelSpeeds.SHOOT_HIGH)) {
      setMainRawWheelSpeed(IntakeWheelSpeeds.EJECT);
      setHorizontalRawWheelSpeed(IntakeWheelSpeeds.EJECT);
    } else if (getGamePieceSensor()
        && stateHandler.getDesiredIntakePosition() == IntakePositions.INTAKE
        && stateHandler.getDesiredIntakeWheelSpeed() == IntakeWheelSpeeds.INTAKE) {
      setMainRawWheelSpeed(IntakeWheelSpeeds.GRIP);
      setHorizontalRawWheelSpeed(IntakeWheelSpeeds.GRIP);
    } else {
      setMainRawWheelSpeed(stateHandler.getDesiredIntakeWheelSpeed());
      setHorizontalRawWheelSpeed(
          stateHandler.getDesiredIntakeWheelSpeed());
    }

  }
}
