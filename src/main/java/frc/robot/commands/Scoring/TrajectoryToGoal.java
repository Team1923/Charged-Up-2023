// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Scoring;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.Swerve;
import frc.robot.interfaces.LimelightInterface;
import frc.robot.interfaces.LimelightInterface.SpecificLimelight;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.StateHandler;
import frc.robot.util.PathPlannerUtils.SuppliedSwerveControllerCommand;
import frc.robot.util.StateVariables.HorizontalLocations;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TrajectoryToGoal extends SequentialCommandGroup {
  TrajectoryConfig config = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
      AutoConstants.kMaxAccelerationMetersPerSecondSquared).setKinematics(Swerve.swerveKinematics);

  private SwerveSubsystem swerve;

  public TrajectoryToGoal(SwerveSubsystem swerve) {
    this.swerve = swerve;

    var thetaController = new PIDController(AutoConstants.kPThetaController, 0, 0);

    addCommands(
        new SuppliedSwerveControllerCommand(
            () -> getPathPlannerTrajectory(),
            swerve::getPose,
            Swerve.swerveKinematics,
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            swerve::setModuleStates,
            false,
            swerve,
            swerve));
  }

  public PathPlannerTrajectory getPathPlannerTrajectory() {
    // heading argument = wheel direction
    // holonomic rotation = actual robot heading
    Rotation2d robotRotation = Rotation2d.fromDegrees(swerve.getYawIEEE());
    // Rotation2d robotHeading = new
    // Rotation2d(Math.toRadians(swerve.getYawIEEE()));
    Rotation2d goalRotation = Rotation2d.fromDegrees((robotRotation.getDegrees() > 0 ? 90 : -90));

    Pose2d center = new Pose2d(0.7, 0, goalRotation);
    Pose2d left = new Pose2d(0.7, 0.659, goalRotation);
    Pose2d right = new Pose2d(0.7, -0.659, goalRotation);

    SpecificLimelight limelight = swerve.getCorrectLimelight();

    Pose2d currentRobotPose = swerve.getPose();
    double desiredWheelHeading = 0;

    Pose3d currentAprilTagPose = LimelightInterface.getInstance().getAprilTagPose(limelight);
    Pose2d aprilTagPose = new Pose2d(currentAprilTagPose.getX(), currentAprilTagPose.getY(), new Rotation2d());

    HorizontalLocations desiredHorizontalLocation = StateHandler.getInstance().getCurrentHorizontalLocation();
    Pose2d desiredLocation = left;

    switch (desiredHorizontalLocation) {
      case LEFT:
        desiredLocation = left;
        break;
      case CENTER:
        desiredLocation = center;
        break;
      case RIGHT:
        desiredLocation = right;
        break;
      case RESET:
        desiredLocation = center;
        break;
      default:
        break;
    }

    Pose2d endPose = new Pose2d(aprilTagPose.getX() + desiredLocation.getX(),
        aprilTagPose.getY() + desiredLocation.getY(), goalRotation);

    if (currentRobotPose.getX() - desiredLocation.getY() >= 0) {
      desiredWheelHeading = -Math.PI / 2;
    } else {
      desiredWheelHeading = Math.PI / 2;
    }

    return PathPlanner.generatePath(
        new PathConstraints(3, 3), 
        new PathPoint(new Translation2d(currentRobotPose.getX(), currentRobotPose.getY()),
            new Rotation2d(desiredWheelHeading), robotRotation,
            swerve.getRobotVelocity()), // initial
        new PathPoint(new Translation2d(endPose.getX(), endPose.getY()), new Rotation2d(Math.PI),
          endPose.getRotation())); // final

  }

}
