// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Scoring;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

/** Add your docs here. */
public class SuppliedSwerveControllerCommand extends SwerveControllerCommand {

	public SuppliedSwerveControllerCommand(
		Supplier<Trajectory> tSupplier,
		Supplier<Pose2d> pose,
		SwerveDriveKinematics kinematics,
		PIDController xController,
		PIDController yController,
		ProfiledPIDController thetaController,
		Consumer<SwerveModuleState[]> outputModuleStates,
		Subsystem... requirements) {
		
		super(tSupplier.get(), pose, kinematics, xController, yController, thetaController, outputModuleStates, requirements);
	}

}
