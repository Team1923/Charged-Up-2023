// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Scoring;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.Swerve;
import frc.robot.interfaces.BetterLimelightInterface;
import frc.robot.interfaces.BetterLimelightInterface.SpecificLimelight;
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
            ()->getPathPlannerTrajectory(),
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
    Rotation2d robotHeading = Rotation2d.fromDegrees(swerve.getYawIEEE());
    //Rotation2d robotHeading = new Rotation2d(Math.toRadians(swerve.getYawIEEE()));
    Rotation2d goalHeading = Rotation2d.fromDegrees((robotHeading.getDegrees() > 0 ? 90 : -90));

    Pose2d center = new Pose2d(0.5, 0, goalHeading);
    Pose2d left = new Pose2d(0.5, 0.559, goalHeading);
    Pose2d right = new Pose2d(0.5, -0.559, goalHeading);

    SpecificLimelight limelight = swerve.getCorrectLimelight();
    //Pose3d currentRobotPose = new Pose3d(new Translation3d(-0.5,0,-1.5), new Rotation3d(0,0,0));
    Pose3d currentRobotPose = BetterLimelightInterface.getInstance().getRobotPose3d(limelight);
    double desiredWheelHeading = 0;


    HorizontalLocations desiredHorizontalLocation = StateHandler.getInstance().getCurrentHorizontalLocation();
    Pose2d desiredLocation = left;

    switch(desiredHorizontalLocation) {
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

    
    if (currentRobotPose.getX() - desiredLocation.getY() >= 0) {
      desiredWheelHeading = -Math.PI/2;
    } else {
      desiredWheelHeading = Math.PI/2;
    }


    
    return PathPlanner.generatePath(
        new PathConstraints(3, 3), // stole the values from our WPILib generator
        new PathPoint(new Translation2d(-currentRobotPose.getZ(), currentRobotPose.getX()), new Rotation2d(desiredWheelHeading), robotHeading,
            swerve.getRobotVelocity()), // initial
        //new PathPoint(new Translation2d(-currentRobotPose.getZ(), center.getY()), new Rotation2d(Math.PI), goalHeading), // intermediary
        new PathPoint(new Translation2d(desiredLocation.getX(), desiredLocation.getY()), new Rotation2d(Math.PI), goalHeading)); // final

  }

}
