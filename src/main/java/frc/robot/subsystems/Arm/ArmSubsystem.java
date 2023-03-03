// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.FalconConstants;
import frc.robot.commands.EmergencyCommands.EStopArmCommand;
import frc.robot.util.StateHandler;
import frc.robot.util.StateVariables.ArmPositions;
import frc.robot.util.StateVariables.CurrentRobotDirection;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  private WPI_TalonFX proximalMotor = new WPI_TalonFX(ArmConstants.proximalMotorID, "Default Name");
  private WPI_TalonFX distalMotor = new WPI_TalonFX(ArmConstants.distalMotorID, "Default Name");
  private DutyCycleEncoder proximalEncoder = new DutyCycleEncoder(ArmConstants.proximalEncoderID); // change this
  private DutyCycleEncoder distalEncoder = new DutyCycleEncoder(ArmConstants.distalEncoderID);
  private StateHandler stateHandler = StateHandler.getInstance();

  private Map<Integer, ArmTrajectory> allTrajectories = new HashMap<>();
  private ArmTrajectory currentTrajectory = null;
  private ArmPose setpointPose = null; // Pose to revert to when not following trajectory
  private ArmPose queuedPose = null; // Use as setpoint once trajectory is completed
  private Timer trajectoryTimer = new Timer();

  static final String configFilename = "arm_config.json";
  private final ArmSolver solver;
  private final String configJson;
  private final ArmConfig config;
  private final ArmKinematics kinematics;
  private final ArmDynamics dynamics;

  public ArmSubsystem(ArmSolver solver) {

    this.solver = solver;

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

    // distalMotor.setSelectedSensorPosition(0);
    resetDistalPosition();
    resetProximalPosition();

  }

  public void resetProximalPosition() {
    proximalMotor.setSelectedSensorPosition(
        (getProximalAbsoluteEncoderRads() - ArmConstants.proximalEncoderZero + ArmConstants.proximalHardstop)
            * ArmConstants.proximalRadsToTicks);
  }

  public void resetDistalPosition() {
    distalMotor.setSelectedSensorPosition(
        (getDistalAbsoluteEncoderRads() - ArmConstants.distalEncoderZero + ArmConstants.distalHardstop
            - Math.toRadians(0))
            * ArmConstants.distalRadsToTicks);
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
    return -distalEncoder.getAbsolutePosition() * ArmConstants.distalAbsoluteEncoderToRadians;
  }

  public double getProximalAbsoluteEncoderTicks() {
    return proximalEncoder.getAbsolutePosition() * ArmConstants.proximalAbsoluteEncoderToTicks;
  }

  public double getDistalAbsoluteEncoderTicks() {
    return distalEncoder.getAbsolutePosition() * ArmConstants.distalAbsoluteEncoderToTicks;
  }

  public boolean isReflected() {
    /*
     * reflection of angles is based on robot direction
     */
    return stateHandler.getRobotDirection() == CurrentRobotDirection.LEFT;
  }

  public double getDistalCurrent() {
    return distalMotor.getStatorCurrent();
  }

  public double getProximalCurrent() {
    return proximalMotor.getStatorCurrent();
  }

  public void disableMotionMagic() {
    proximalMotor.set(ControlMode.Disabled, 0);
    distalMotor.set(ControlMode.Disabled, 0);
  }

  @Override
  public void periodic() {

    if (getDistalCurrent() > 100 || getProximalCurrent() > 100) { // FIND CURRENT VALUES THAT WORK
      proximalMotor.stopMotor();
      distalMotor.stopMotor();
      CommandScheduler.getInstance().schedule(new EStopArmCommand(this));
    }

    





  }


}
