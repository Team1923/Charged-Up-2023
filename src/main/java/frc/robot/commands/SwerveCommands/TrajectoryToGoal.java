// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SwerveCommands;

import java.util.ArrayList;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.Swerve;
import frc.robot.interfaces.LimelightInterface;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.StateHandler;
import frc.robot.util.PathPlannerUtils.SuppliedSwerveControllerCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TrajectoryToGoal extends SequentialCommandGroup {
  private SwerveSubsystem swerve;

  public TrajectoryToGoal(SwerveSubsystem s) {
    swerve = s;
    PIDController thetaController = new PIDController(AutoConstants.kPThetaController, 0, 0);

    addCommands(
        new SuppliedSwerveControllerCommand(
            () -> getPPTrajectory(),
            swerve::getPose,
            Swerve.swerveKinematics,
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            swerve::setModuleStates,
            false,
            swerve,
            swerve),
        new InstantCommand(() -> StateHandler.getInstance().setReadyToScore(true))

    );
  }

  public PathPlannerTrajectory getPPTrajectory() {
    Rotation2d currentHolonomicRotation = Rotation2d.fromDegrees(swerve.getYawIEEE());
    Rotation2d goalHolonomicRotation = new Rotation2d(0);

    Pose2d currentRobotPose = swerve.getPose();

    Pose2d closestTag = getMinPose(currentRobotPose, DriverStation.getAlliance());

    Rotation2d desiredWheelHeading = getWheelHeading(currentRobotPose, closestTag);

    return PathPlanner.generatePath(
        new PathConstraints(3, 3),
        new PathPoint(new Translation2d(currentRobotPose.getX(), currentRobotPose.getY()),
            desiredWheelHeading, currentHolonomicRotation,
            swerve.getRobotVelocity()), // initial
        new PathPoint(new Translation2d(closestTag.getX(), closestTag.getY()), desiredWheelHeading,
            goalHolonomicRotation)); // final

  }

  public Pose2d getMinPose(Pose2d robot, Alliance a) {
    double dist;
    double minDistance = 999999;
    Pose2d returnPose = new Pose2d();
    ArrayList<AprilTag> aprilTags = LimelightInterface.getInstance().getAprilTagList();
    if (a == Alliance.Red) {
      for (int i = 0; i <= 2; i++) {
        dist = Math.abs(robot.getY() - aprilTags.get(i).pose.getY());
        if (dist < minDistance) {
          minDistance = dist;
          returnPose = new Pose2d(aprilTags.get(i).pose.getX(), aprilTags.get(i).pose.getY(), new Rotation2d());
        }
      }
    } else {
      for (int i = 5; i <= 7; i++) {
        dist = Math.abs(robot.getY() - aprilTags.get(i).pose.getY());
        if (dist < minDistance) {
          minDistance = dist;
          returnPose = new Pose2d(aprilTags.get(i).pose.getX(), aprilTags.get(i).pose.getY(), new Rotation2d());
        }
      }
    }

    return returnPose;
  }

  public Rotation2d getWheelHeading(Pose2d currentRobotPose, Pose2d closestTag) {
    double dx = closestTag.getX() - currentRobotPose.getX();
    double dy = closestTag.getY() - currentRobotPose.getY();

    if (dx >= 0 && dy >= 0) {
      return new Rotation2d(Math.atan(dy / dx));
    } else if (dx >= 0 && dy <= 0) {
      return new Rotation2d(Math.atan(dy / dx));
    } else if (dx <= 0 && dy >= 0) {
      return new Rotation2d(Math.atan(dy / dx) + Math.PI);
    } else {
      return new Rotation2d(Math.atan(dy / dx) - Math.PI);
    }
  }

}
