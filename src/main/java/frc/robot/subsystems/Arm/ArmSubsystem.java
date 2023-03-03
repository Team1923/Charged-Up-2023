// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.FalconConstants;
import frc.robot.commands.EmergencyCommands.EStopArmCommand;
import frc.robot.subsystems.Arm.ArmSolver.ArmSolverIOInputs;
import frc.robot.util.StateHandler;
import frc.robot.util.StateVariables.CurrentRobotDirection;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  private WPI_TalonFX proximalMotor = new WPI_TalonFX(ArmConstants.proximalMotorID, "Default Name");
  private WPI_TalonFX distalMotor = new WPI_TalonFX(ArmConstants.distalMotorID, "Default Name");
  private DutyCycleEncoder proximalEncoder = new DutyCycleEncoder(ArmConstants.proximalEncoderID); // change this
  private DutyCycleEncoder distalEncoder = new DutyCycleEncoder(ArmConstants.distalEncoderID);
  private StateHandler stateHandler = StateHandler.getInstance();

  private final double trajectoryCacheMarginRadians = 0.02;
  private final boolean forcePregenerated = false;

  private Supplier<Boolean> disableSupplier = () -> false;

  private double proximalAngle = 0;
  private double distalAngle = 0;

  private PIDController shoulderFeedback = new PIDController(0.0, 0.0, 0.0, 0.02);
  private PIDController elbowFeedback = new PIDController(0.0, 0.0, 0.0, 0.02);

  private Map<Integer, ArmTrajectory> allTrajectories = new HashMap<>();
  private ArmTrajectory currentTrajectory = null;
  private ArmPose setpointPose = null; // Pose to revert to when not following trajectory
  private ArmPose queuedPose = null; // Use as setpoint once trajectory is
  // completed
  private Timer trajectoryTimer = new Timer();
  private boolean presetMessagePrinted = false;
  private int presetTrajectoryCount = 0;

  static final String configFilename = "arm_config.json";
  private final ArmSolver solver;
  private final String configJson;
  private final ArmConfig config;
  private final ArmKinematics kinematics;
  private final ArmDynamics dynamics;
  private final ArmSolverIOInputs solverInputs = new ArmSolverIOInputs();

  public ArmSubsystem(ArmSolver solver) {

    this.solver = solver;

    File configFile = new File(Filesystem.getDeployDirectory(), configFilename);
    try {
      configJson = Files.readString(configFile.toPath());
    } catch (IOException e) {
      throw new RuntimeException("Failed to read raw arm config JSON");
    }
    config = ArmConfig.loadJson(configFile);
    solver.setConfig(configJson);
    kinematics = new ArmKinematics(config);
    dynamics = new ArmDynamics(config);
    ArmPose.Preset.updateStowPreset(config);

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

    // Get current arm angles for use
    proximalAngle = getProximalPosition();
    distalAngle = getDistalPosition();

    // Get new trajectory from solver if available
    if (solverInputs.parameterHash != 0
        && allTrajectories.containsKey((int) solverInputs.parameterHash)) {
      var trajectory = allTrajectories.get((int) solverInputs.parameterHash);
      if (!trajectory.isGenerated()) {
        List<Vector<N2>> points = new ArrayList<>();
        for (int i = 0; i < solverInputs.shoulderPoints.length; i++) {
          points.add(VecBuilder.fill(solverInputs.shoulderPoints[i], solverInputs.elbowPoints[i]));
        }
        trajectory.setPoints(solverInputs.totalTime, points);
      }
    }

    // Request next trajectory from solver
    updateTrajectoryRequest();

    // Log status of cached trajectories
    int trajectoryCount = allTrajectories.size();
    SmartDashboard.putNumber("Total Trajectory Count: ", trajectoryCount);
    int trajectoryCountGenerated = 0;
    for (var trajectory : allTrajectories.values()) {
      if (trajectory.isGenerated())
        trajectoryCountGenerated++;
    }

    if (trajectoryCountGenerated >= presetTrajectoryCount) {
      if (!presetMessagePrinted) {
        System.out.println("All preset arm trajectories ready!");
        presetMessagePrinted = true;
      }
    }

    // Set setpoint to current position when disabled (don't move when enabling)
    if (DriverStation.isDisabled()) {
      setpointPose = new ArmPose(
          kinematics.forward(VecBuilder.fill(proximalAngle, distalAngle)));
      currentTrajectory = null;
      queuedPose = null;
    }

    // Check if trajectory is finished
    if (currentTrajectory != null
        && currentTrajectory.isGenerated()
        && trajectoryTimer.hasElapsed(currentTrajectory.getTotalTime())) {
      trajectoryTimer.stop();
      trajectoryTimer.reset();
      currentTrajectory = null;
      setpointPose = queuedPose;
    }

    // Run shoulder and elbow
    if (DriverStation.isDisabled() || disableSupplier.get()) {
      // Stop moving when disabled
      setProximalVoltage(0.0);
      setDistalVoltage(0.0);
      shoulderFeedback.reset();
      elbowFeedback.reset();

    } else if (currentTrajectory != null && currentTrajectory.isGenerated()) {
      // Follow trajectory
      trajectoryTimer.start();
      var state = currentTrajectory.sample(trajectoryTimer.get());
      var voltages = dynamics.feedforward(state);
      setProximalVoltage(
          voltages.get(0, 0) + shoulderFeedback.calculate(proximalAngle, state.get(0, 0)));
      setDistalVoltage(voltages.get(1, 0) + elbowFeedback.calculate(distalAngle, state.get(1, 0)));
      setpointPose = // If trajectory is interrupted, go to last setpoint
          new ArmPose(
              kinematics.forward(new Vector<>(state.extractColumnVector(0))));
    } else {
      // Go to setpoint
      Optional<Vector<N2>> angles = kinematics.inverse(setpointPose.endEffectorPosition());
      if (angles.isPresent()) {
        var voltages = dynamics.feedforward(angles.get());
        setProximalVoltage(
            voltages.get(0, 0) + shoulderFeedback.calculate(proximalAngle, angles.get().get(0, 0)));
        setDistalVoltage(
            voltages.get(1, 0) + elbowFeedback.calculate(distalAngle, angles.get().get(1, 0)));
      } else {
        setProximalVoltage(0.0);
        setDistalVoltage(0.0);
      }
    }

  }

  /** Returns whether the current current is complete. */
  public boolean isTrajectoryFinished() {
    return currentTrajectory == null;
  }

  /** Starts navigating to a pose. */
  public void runPath(ArmPose.Preset preset) {
    runPath(preset.getPose());
  }

  /** Starts navigating to a pose. */
  public void runPath(ArmPose pose) {
    // Get current and target angles
    Optional<Vector<N2>> currentAngles = kinematics.inverse(setpointPose.endEffectorPosition());
    Optional<Vector<N2>> targetAngles = kinematics.inverse(pose.endEffectorPosition());
    if (currentAngles.isEmpty() || targetAngles.isEmpty()) {
      return;
    }

    // Exit if already at setpoint
    if (Math.abs(currentAngles.get().get(0, 0) - targetAngles.get().get(0, 0)) < trajectoryCacheMarginRadians
        && Math.abs(currentAngles.get().get(1, 0) - targetAngles.get().get(1, 0)) < trajectoryCacheMarginRadians) {
      currentTrajectory = null;
      setpointPose = pose;
      return;
    }

    // Create parameters
    var parameters = new TrajectoryParameters(currentAngles.get(), targetAngles.get());

    // Reset current trajectory
    trajectoryTimer.stop();
    trajectoryTimer.reset();

    // Search for similar trajectories
    ArmTrajectory closestTrajectory = null;
    double closestTrajectoryDiff = Double.POSITIVE_INFINITY;
    for (var trajectory : allTrajectories.values()) {
      var initialDiff = trajectory
          .getParameters()
          .initialJointPositions()
          .minus(parameters.initialJointPositions());
      var finalDiff = trajectory.getParameters().finalJointPositions().minus(parameters.finalJointPositions());

      // Check if closest
      double totalDiff = Math.abs(initialDiff.get(0, 0))
          + Math.abs(initialDiff.get(1, 0))
          + Math.abs(finalDiff.get(0, 0))
          + Math.abs(finalDiff.get(1, 0));
      if (totalDiff < closestTrajectoryDiff) {
        closestTrajectory = trajectory;
        closestTrajectoryDiff = totalDiff;
      }

      // If close enough, use this trajectory
      if (Math.abs(initialDiff.get(0, 0)) < trajectoryCacheMarginRadians
          && Math.abs(initialDiff.get(1, 0)) < trajectoryCacheMarginRadians
          && Math.abs(finalDiff.get(0, 0)) < trajectoryCacheMarginRadians
          && Math.abs(finalDiff.get(1, 0)) < trajectoryCacheMarginRadians
          && trajectory.isGenerated()) {
        currentTrajectory = trajectory;
        queuedPose = pose;
        return;
      }
    }

    // Use closest if overriden
    if (forcePregenerated) {
      if (closestTrajectory != null) {
        currentTrajectory = closestTrajectory;
        queuedPose = pose;
      }
      return;
    }

    // Create new trajectory
    var trajectory = new ArmTrajectory(parameters);
    allTrajectories.put(parameters.hashCode(), trajectory);
    currentTrajectory = trajectory;
    queuedPose = pose;
    updateTrajectoryRequest(); // Start solving immediately if nothing else in queue
  }

  /**
   * Finds the next ungenerated trajectory and request it from the solver. If that
   * trajectory is
   * already being generated this will have no effect.
   */
  private void updateTrajectoryRequest() {
    for (var trajectory : allTrajectories.values()) {
      if (!trajectory.isGenerated()) {
        solver.request(trajectory.getParameters());
        break;
      }
    }
  }

}
