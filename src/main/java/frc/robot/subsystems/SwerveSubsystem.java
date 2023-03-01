// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.interfaces.BetterLimelightInterface;
import frc.robot.interfaces.SwerveModule;
import frc.robot.interfaces.BetterLimelightInterface.SpecificLimelight;
import frc.robot.util.StateHandler;
import frc.robot.util.StateVariables.CurrentRobotDirection;
import frc.robot.Constants;
import frc.robot.Constants.Swerve;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.ArrayList;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveSubsystem extends SubsystemBase {
    public SwerveDrivePoseEstimator swerveOdometry;
    public SwerveModule[] mSwerveMods;

    public Pigeon2 gyro = new Pigeon2(Swerve.pigeonID, "rio");

    private Field2d field2D = new Field2d();

    private StateHandler stateHandler = StateHandler.getInstance();

    public static final Vector<N3> stateStdDevs = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(5));

    public static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(1, 1, Units.degreesToRadians(10));

    BetterLimelightInterface limelightInterface = BetterLimelightInterface.getInstance();

    public SwerveSubsystem() {
        gyro.configFactoryDefault();
        zeroGyro();

        mSwerveMods = new SwerveModule[] {
                new SwerveModule(0, Constants.Swerve.Mod0.constants),
                new SwerveModule(1, Constants.Swerve.Mod1.constants),
                new SwerveModule(2, Constants.Swerve.Mod2.constants),
                new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        /*
         * By pausing init for a second before setting module offsets, we avoid a bug
         * with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0);
        resetModulesToAbsolute();

        swerveOdometry = new SwerveDrivePoseEstimator(Swerve.swerveKinematics, getYaw(), getModulePositions(),
                new Pose2d(1.69, 4.96, new Rotation2d(0)),
                stateStdDevs, visionMeasurementStdDevs);

        SmartDashboard.putData(field2D);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        getYaw())
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public Pose2d getPose() {
        return swerveOdometry.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    // Return the speed of the robot using the sum of the X and Y chassis speeds
    // taken from
    // a forward kinematics calculation from the swerve drive kinematics object
    public double getRobotVelocity() {
        ChassisSpeeds currentChassisSpeeds = Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());

        return Math.sqrt(Math.pow(currentChassisSpeeds.vxMetersPerSecond, 2)
                + Math.pow(currentChassisSpeeds.vyMetersPerSecond, 2));
    }

    public void zeroGyro() {
        gyro.setYaw(0);
    }

    public void zeroGyro(double yaw) {
        gyro.setYaw(yaw);
    }

    public Rotation2d getYaw() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(-getYawIEEE())
                : Rotation2d.fromDegrees(getYawIEEE());
    }

    public double getYawIEEE() {
        return Math.IEEEremainder(gyro.getYaw(), 360);
    }

    public void resetModulesToAbsolute() {
        for (SwerveModule mod : mSwerveMods) {
            mod.resetToAbsolute();
        }
    }

    public void stop() {
        for (SwerveModule mod : mSwerveMods) {
            mod.stop();
        }
    }

    public SpecificLimelight getCorrectLimelight() {
        if (getYawIEEE() >= 0 && getYawIEEE() <= 180) {
            return SpecificLimelight.LEFT_LIMELIGHT;
        } else {
            return SpecificLimelight.RIGHT_LIMELIGHT;
        }
    }

    public void updateOdometry() {
        swerveOdometry.update(Rotation2d.fromDegrees(getYawIEEE()), getModulePositions());
        if (limelightInterface.hasValidTargets(getCorrectLimelight())) {
            Pose3d currentAprilTagPose = limelightInterface.getAprilTagPose(getCorrectLimelight());
            Pose2d aprilTagPose = new Pose2d(currentAprilTagPose.getX(), currentAprilTagPose.getY(), new Rotation2d());
            Pose2d robotLimelightPose = new Pose2d(-limelightInterface.getRobotPose3d(getCorrectLimelight()).getZ(),
                    limelightInterface.getRobotPose3d(getCorrectLimelight()).getX(), getYaw());
            if (Math.sqrt(Math.pow(robotLimelightPose.getX(), 2) + Math.pow(robotLimelightPose.getY(), 2)) <= 1) {
                Pose2d newRobotPose = new Pose2d(aprilTagPose.getX() + robotLimelightPose.getX(),
                        aprilTagPose.getY() + robotLimelightPose.getY(), getYaw());
                swerveOdometry.addVisionMeasurement(newRobotPose,
                        Timer.getFPGATimestamp() - limelightInterface.getBotPose(getCorrectLimelight())[5] / 1000);
            }

        }
    }

    @Override
    public void periodic() {
        updateOdometry();
        if (getCorrectLimelight() == SpecificLimelight.LEFT_LIMELIGHT) {
            stateHandler.setRobotDirection(CurrentRobotDirection.LEFT);
        } else {
            stateHandler.setRobotDirection(CurrentRobotDirection.RIGHT);
        }
        SmartDashboard.putString("CURRENT ROBOT DIRECTION", stateHandler.getRobotDirection().toString());

        SmartDashboard.putString("ROBOT ODOMETRY", swerveOdometry.getEstimatedPosition().toString());

        // PathPlannerServer.sendPathFollowingData(new Pose2d(), swerveOdometry.getEstimatedPosition());

        // for(SwerveModule mod : mSwerveMods){
        // SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder",
        // mod.getCanCoder().getDegrees());
        // SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated",
        // mod.getPosition().angle.getDegrees());
        // SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity",
        // mod.getState().speedMetersPerSecond);
        // }

        // field2D.setRobotPose(getPose());

        // SmartDashboard.putString("CORRECT LIMELIGHT",
        // getCorrectLimelight().toString());
        // SmartDashboard.putNumber("YAW IEEE", getYawIEEE());
        // SmartDashboard.putNumber("left limelight botpose",
        // BetterLimelightInterface.getInstance().getTargetArea(getCorrectLimelight()));
    }
}
