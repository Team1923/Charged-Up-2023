// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
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
import frc.robot.util.StateHandler;
import frc.robot.util.StateVariables.GamePieceMode;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private WPI_TalonFX intakeArmMaster = new WPI_TalonFX(IntakeConstants.intakeProximalID, "rio");
  private WPI_TalonFX intakeArmFollower = new WPI_TalonFX(IntakeConstants.intakeDistalID, "rio");
  private WPI_TalonFX leftIntakeWheelMotor = new WPI_TalonFX(IntakeConstants.leftIntakeWheelMotor, "rio");
  private WPI_TalonFX rightIntakeWheelMotor = new WPI_TalonFX(IntakeConstants.rightIntakeWheelMotor, "rio");


  private DutyCycleEncoder intakeEncoder = new DutyCycleEncoder(
      IntakeConstants.intakeProximalAbsoluteEncoderID);

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

    intakeArmMaster.config_kP(0, IntakeConstants.intakeProximalkP, FalconConstants.timeoutMs);
    intakeArmMaster.config_kI(0, IntakeConstants.intakeProximalkI, FalconConstants.timeoutMs);
    intakeArmMaster.config_kD(0, IntakeConstants.intakeProximalkD, FalconConstants.timeoutMs);

    intakeArmFollower.config_kP(0, IntakeConstants.intakeProximalkP, FalconConstants.timeoutMs);
    intakeArmFollower.config_kI(0, IntakeConstants.intakeProximalkI, FalconConstants.timeoutMs);
    intakeArmFollower.config_kD(0, IntakeConstants.intakeProximalkD, FalconConstants.timeoutMs);

    intakeArmMaster.configMotionCruiseVelocity(IntakeConstants.maxIntakeProximalVel);
    intakeArmMaster.configMotionAcceleration(IntakeConstants.maxIntakeProximalAccel);
    intakeArmFollower.configMotionCruiseVelocity(IntakeConstants.maxIntakeDistalVel);
    intakeArmFollower.configMotionAcceleration(IntakeConstants.maxIntakeDistalAccel);

    intakeArmMaster.setNeutralMode(NeutralMode.Coast);
    intakeArmFollower.setNeutralMode(NeutralMode.Coast);
    leftIntakeWheelMotor.setNeutralMode(NeutralMode.Brake);
    rightIntakeWheelMotor.setNeutralMode(NeutralMode.Brake);

    resetIntakePosition();

    leftIntakeWheelMotor.setInverted(InvertType.InvertMotorOutput);
    rightIntakeWheelMotor.setInverted(InvertType.InvertMotorOutput);

  }


  //For both the Proximal and Distal, using the Encoders and physical hardstops, we can reset their position
  

  public void resetIntakePosition() {
    double setZeroPosition = (getIntakeAbsoluteEncoderRads() - IntakeConstants.distalEncoderZero + IntakeConstants.intakeDistalHardstop)
    * IntakeConstants.intakeDistalRadsToTicks;
    intakeArmMaster.setSelectedSensorPosition(setZeroPosition);

    intakeArmFollower.setSelectedSensorPosition(setZeroPosition);
  }


  public void setIntakePosition(double distalAngle) {
    intakeArmMaster.set(ControlMode.MotionMagic, distalAngle * IntakeConstants.intakeDistalRadsToTicks,
        DemandType.ArbitraryFeedForward, calculateIntakeFeedforward());
    intakeArmFollower.set(ControlMode.MotionMagic, distalAngle * IntakeConstants.intakeDistalRadsToTicks,
    DemandType.ArbitraryFeedForward, calculateIntakeFeedforward());
  }


  public double getIntakeArmPosition() {
    return ((intakeArmMaster.getSelectedSensorPosition() * IntakeConstants.intakeDistalTicksToRad)
        + IntakeConstants.kIntakeDistalOffsetRads);
  }

  public double getIntakeAbsoluteEncoderRads() {
    return (intakeEncoder.getAbsolutePosition() *
        IntakeConstants.intakeProximalAbsoluteEncoderToRadians);
  }

  

  public double getIntakeProximalAbsoluteEncoderTicks() {
    return (intakeEncoder.getAbsolutePosition() *
        IntakeConstants.intakeProximalAbsoluteEncoderToTicks);
  }




  public double calculateIntakeFeedforward() {
    return IntakeConstants.intakeMaxDistalGravityConstant * Math.cos(getIntakeArmPosition());
  }

  //Using measured constants, we are able to find the angle from the C.O.M, with respect to the horizontal
  
  
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

  public void stopIntake(){
    intakeArmMaster.stopMotor();
  }

  public void disableMotionMagic() {
    intakeArmMaster.set(ControlMode.Disabled, 0);
  }

  @Override
  public void periodic() {

    // if(DriverStation.isDisabled()) {
    //   disableMotionMagic();
    // }

    //If the current for either the proximal or distal is above 50, we stop the motors and stop running the intake subsystem
    // if(getIntakeProximalCurrent() > 100 || getIntakeDistalCurrent() > 100){
    //   intakeProximalMotor.stopMotor();
    //   intakeDistalMotor.stopMotor();
    //   CommandScheduler.getInstance().schedule(new EStopIntakeCommand(this));
    // }

    double intakeError = Math
        .abs(getIntakeArmPosition() - stateHandler.getDesiredIntakePosition().getArmAngles().getProximalAngle());

    boolean withinThreshold = intakeError < IntakeConstants.errorThreshold;
       
   
    if (withinThreshold) {
      stateHandler.setCurrentIntakePosition(stateHandler.getDesiredIntakePosition());
    }


    SmartDashboard.putString("DESIRED INTAKE State", stateHandler.getDesiredIntakePosition().toString());
    SmartDashboard.putString("CURRENT INTAKE State", stateHandler.getCurrentIntakePosition().toString());

    SmartDashboard.putNumber("INTAKE Arm POSITION RADS: ", getIntakeArmPosition());
  

    SmartDashboard.putNumber("INTAKE ABSOLUTE  Encoder Rads", getIntakeAbsoluteEncoderRads());
    
    // SmartDashboard.putString("Desired Intake Position", stateHandler.getDesiredIntakePosition().toString());
    // SmartDashboard.putString("Current Intake Position", stateHandler.getCurrentIntakePosition().toString());


    // SmartDashboard.putBoolean("LIMIT SWITCH", getGamePieceSensor());

    // SmartDashboard.putString("Scoring Location Vertical", stateHandler.getCurrentVerticalLocation().toString());
    // SmartDashboard.putString("Scoring Location Horizontal", stateHandler.getCurrentHorizontalLocation().toString());

    // SmartDashboard.putNumber("Proixmal Closed Loop Error", intakeProximalMotor.getSelectedSensorPosition() - intakeProximalMotor.getClosedLoopTarget());
    // SmartDashboard.putNumber("Distal Closed Loop Error", intakeDistalMotor.getSelectedSensorPosition() - intakeDistalMotor.getClosedLoopTarget());

  }
}
