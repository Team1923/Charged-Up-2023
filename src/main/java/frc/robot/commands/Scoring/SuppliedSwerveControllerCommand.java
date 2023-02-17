// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Scoring;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * A command that uses two PID controllers ({@link PIDController}) and a
 * ProfiledPIDController
 * ({@link ProfiledPIDController}) to follow a trajectory {@link Trajectory}
 * with a swerve drive.
 *
 * <p>
 * This command outputs the raw desired Swerve Module States
 * ({@link SwerveModuleState}) in an
 * array. The desired wheel and module rotation velocities should be taken from
 * those and used in
 * velocity PIDs.
 *
 * <p>
 * The robot angle controller does not follow the angle given by the trajectory
 * but rather goes
 * to the angle given in the final state of the trajectory.
 *
 * <p>
 * This class is provided by the NewCommands VendorDep
 */
public class SuppliedSwerveControllerCommand extends CommandBase {
	private Timer m_timer = new Timer();
	private Trajectory m_trajectory;
	private Supplier<Pose2d> m_pose;
	private SwerveDriveKinematics m_kinematics;
	private HolonomicDriveController m_controller;
	private Consumer<SwerveModuleState[]> m_outputModuleStates;
	private Supplier<Rotation2d> m_desiredRotation;
	private Supplier<Trajectory> trajectorySupplier;
	private SwerveSubsystem swerve;

	// Trajectory Supplier Version
	public SuppliedSwerveControllerCommand(
			Supplier<Trajectory> tSupplier,
			Supplier<Pose2d> pose,
			SwerveDriveKinematics kinematics,
			PIDController xController,
			PIDController yController,
			ProfiledPIDController thetaController,
			Consumer<SwerveModuleState[]> outputModuleStates,
			SwerveSubsystem requirements) {

		this(
				new Trajectory(),
				pose,
				kinematics,
				xController,
				yController,
				thetaController,
				() -> new Trajectory().getStates().get(new Trajectory().getStates().size() - 1).poseMeters.getRotation(),
				outputModuleStates,
				requirements);

		this.trajectorySupplier = tSupplier;
	}

	public SuppliedSwerveControllerCommand(
			Trajectory trajectory,
			Supplier<Pose2d> pose,
			SwerveDriveKinematics kinematics,
			PIDController xController,
			PIDController yController,
			ProfiledPIDController thetaController,
			Supplier<Rotation2d> desiredRotation,
			Consumer<SwerveModuleState[]> outputModuleStates,
			SwerveSubsystem requirements) {
		this(
				trajectory,
				pose,
				kinematics,
				new HolonomicDriveController(
						requireNonNullParam(xController, "xController", "SwerveControllerCommand"),
						requireNonNullParam(yController, "yController", "SwerveControllerCommand"),
						requireNonNullParam(thetaController, "thetaController", "SwerveControllerCommand")),
				desiredRotation,
				outputModuleStates,
				requirements);
	}

	public SuppliedSwerveControllerCommand(
			Trajectory trajectory,
			Supplier<Pose2d> pose,
			SwerveDriveKinematics kinematics,
			PIDController xController,
			PIDController yController,
			ProfiledPIDController thetaController,
			Consumer<SwerveModuleState[]> outputModuleStates,
			SwerveSubsystem requirements) {
		this(
				trajectory,
				pose,
				kinematics,
				xController,
				yController,
				thetaController,
				() -> trajectory.getStates().get(trajectory.getStates().size() - 1).poseMeters.getRotation(),
				outputModuleStates,
				requirements);
	}

	public SuppliedSwerveControllerCommand(
			Trajectory trajectory,
			Supplier<Pose2d> pose,
			SwerveDriveKinematics kinematics,
			HolonomicDriveController controller,
			Consumer<SwerveModuleState[]> outputModuleStates,
			SwerveSubsystem requirements) {
		this(
				trajectory,
				pose,
				kinematics,
				controller,
				() -> trajectory.getStates().get(trajectory.getStates().size() - 1).poseMeters.getRotation(),
				outputModuleStates,
				requirements);
	}

	public SuppliedSwerveControllerCommand(
			Trajectory trajectory,
			Supplier<Pose2d> pose,
			SwerveDriveKinematics kinematics,
			HolonomicDriveController controller,
			Supplier<Rotation2d> desiredRotation,
			Consumer<SwerveModuleState[]> outputModuleStates,
			SwerveSubsystem requirements) {
		m_trajectory = requireNonNullParam(trajectory, "trajectory", "SwerveControllerCommand");
		m_pose = requireNonNullParam(pose, "pose", "SwerveControllerCommand");
		m_kinematics = requireNonNullParam(kinematics, "kinematics", "SwerveControllerCommand");
		m_controller = requireNonNullParam(controller, "controller", "SwerveControllerCommand");

		m_desiredRotation = requireNonNullParam(desiredRotation, "desiredRotation", "SwerveControllerCommand");

		m_outputModuleStates = requireNonNullParam(outputModuleStates, "outputModuleStates", "SwerveControllerCommand");

		addRequirements(requirements);

		this.swerve = requirements;
	}

	@Override
	public void initialize() {

		m_trajectory = trajectorySupplier.get();

		swerve.resetOdometry(m_trajectory.getInitialPose());

		m_controller.setTolerance(new Pose2d(new Translation2d(.1,.1), new Rotation2d(.1)));
		m_desiredRotation = () -> m_trajectory.getStates().get(m_trajectory.getStates().size() - 1).poseMeters.getRotation();

		m_timer.reset();
		m_timer.start();
	}

	@Override
	public void execute() {
		double curTime = m_timer.get();
		var desiredState = m_trajectory.sample(curTime);

		var targetChassisSpeeds = m_controller.calculate(m_pose.get(), desiredState, m_desiredRotation.get());
		var targetModuleStates = m_kinematics.toSwerveModuleStates(targetChassisSpeeds);

		m_outputModuleStates.accept(targetModuleStates);
	}

	@Override
	public void end(boolean interrupted) {
		m_timer.stop();
	}

	@Override
	public boolean isFinished() {
		return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds()) && m_controller.atReference();
	}
}
