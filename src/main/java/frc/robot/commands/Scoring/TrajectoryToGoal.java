// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Scoring;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.Swerve;
import frc.robot.interfaces.BetterLimelightInterface;
import frc.robot.interfaces.BetterLimelightInterface.SpecificLimelight;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TrajectoryToGoal extends SequentialCommandGroup {
  TrajectoryConfig config = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
      AutoConstants.kMaxAccelerationMetersPerSecondSquared).setKinematics(Swerve.swerveKinematics);

  private SwerveSubsystem swerve;

  public TrajectoryToGoal(SwerveSubsystem swerve) {
    this.swerve = swerve;

    var thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0.001,
        AutoConstants.kThetaControllerConstraints);

    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    addCommands(
        new SuppliedSwerveControllerCommand(
            () -> getPathPlannerTrajectory(),
            swerve::getPose,
            Swerve.swerveKinematics,
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            swerve::setModuleStates,
            swerve));
  }

  public PathPlannerTrajectory getPathPlannerTrajectory() {
    // heading argument = wheel direction
    // holonomic rotation = actual robot heading
    Rotation2d robotHeading = new Rotation2d(Math.toRadians(swerve.getYawIEEE()));
    Rotation2d goalHeading = new Rotation2d(Math.toRadians(swerve.getYawIEEE() > 0 ? 90 : -90));
    Rotation2d wheelHeading = swerve.getHeadingFromChassisSpeed(); //this is already given in radians

    Pose2d center = new Pose2d(0.5, 0, goalHeading);
    Pose2d left = new Pose2d(0.5, -0.55, goalHeading);
    Pose2d right = new Pose2d(0.2, 0.55, goalHeading);

    SpecificLimelight limelight = swerve.getCorrectLimelight();
    Pose3d currentRobotPose = BetterLimelightInterface.getInstance().getRobotPose3d(limelight);

    return PathPlanner.generatePath(
        new PathConstraints(3, 3), // stole the values from our WPILib generator
        new PathPoint(new Translation2d(-currentRobotPose.getZ(), currentRobotPose.getX()), wheelHeading, robotHeading,
            swerve.getRobotVelocity()), // initial
        new PathPoint(new Translation2d(-currentRobotPose.getZ(), center.getY()), new Rotation2d(Math.PI)), // intermediary
        new PathPoint(new Translation2d(center.getX(), center.getY()), new Rotation2d(Math.PI), goalHeading)); // final

  }

  public Trajectory generateTrajectory() {

    double robotAngle = swerve.getYawIEEE();

    double goalAngle = robotAngle > 0 ? 90 : -90;

    Pose2d center = new Pose2d(0.5, 0, new Rotation2d(Math.toRadians(goalAngle)));
    Pose2d left = new Pose2d(0.5, -0.55, new Rotation2d((Math.toRadians(goalAngle))));
    Pose2d right = new Pose2d(0.2, 0.55, new Rotation2d((Math.toRadians(goalAngle))));

    SpecificLimelight limelight = swerve.getCorrectLimelight();

    Pose3d currentRobotPose = BetterLimelightInterface.getInstance().getRobotPose3d(limelight);

    config.setStartVelocity(swerve.getRobotVelocity());
    config.setEndVelocity(0);
    config.setReversed(false);

    return TrajectoryGenerator.generateTrajectory(
        // current bot pose
        new Pose2d(-currentRobotPose.getZ(), currentRobotPose.getX(), new Rotation2d(Math.toRadians(robotAngle))),
        // intersection point
        List.of(new Translation2d(-currentRobotPose.getZ(), center.getY())),
        // target point
        left, config);
  }

}
