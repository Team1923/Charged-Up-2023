// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.interfaces.SwerveModule;

public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new SwerveSubsystem. */

  /*
   * Create your SwerveModule Objects!
   * Pass in all the necessary constants
   * from the constants file. 
   */
  public final SwerveModule frontLeft = new SwerveModule(
      DriveConstants.kFrontLeftDriveMotorPort,
      DriveConstants.kFrontLeftTurningMotorPort,
      DriveConstants.kFrontLeftDriveReversed,
      DriveConstants.kFrontLeftTurningReversed,
      DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
      DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
      DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetReversed);

  public final SwerveModule frontRight = new SwerveModule(
      DriveConstants.kFrontRightDriveMotorPort,
      DriveConstants.kFrontRightTurningMotorPort,
      DriveConstants.kFrontRightDriveReversed,
      DriveConstants.kFrontRightTurningReversed,
      DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
      DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
      DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetReversed);

  public final SwerveModule backRight = new SwerveModule(
      DriveConstants.kBackRightDriveMotorPort,
      DriveConstants.kBackRightTurningMotorPort,
      DriveConstants.kBackRightDriveReversed,
      DriveConstants.kBackRightTurningReversed,
      DriveConstants.kBackRightDriveAbsoluteEncoderPort,
      DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
      DriveConstants.kBackRightDriveAbsoluteEncoderOffsetReversed);

  public final SwerveModule backLeft = new SwerveModule(
      DriveConstants.kBackLeftDriveMotorPort,
      DriveConstants.kBackLeftTurningMotorPort,
      DriveConstants.kBackLeftDriveReversed,
      DriveConstants.kBackLeftTurningReversed,
      DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
      DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
      DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetReversed);

  private Pigeon2 gyro = new Pigeon2(DriveConstants.pigeonCANID, "Default Name");
  private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, getRotation2d(), getModulePositions());

  public SwerveSubsystem() {
    new Thread(
      () -> {
        try{
          Thread.sleep(1000);
          gyro.configFactoryDefault();
          Thread.sleep(1000);
          zeroHeading();
        } catch(Exception e){}
      }
    ).start();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometer.update(getRotation2d(), getModulePositions());
  }

  public double getHeading(){
    return Math.IEEEremainder(gyro.getYaw(), 360);
  }

  public double getHeading(double offset){
    return -Math.IEEEremainder(gyro.getYaw() + offset, 360);
  }

  public double getRawHeading(){
    return -gyro.getYaw();
  }

  public void zeroHeading(){
    gyro.setYaw(0);
  }

  public Rotation2d getRotation2d(){
    return new Rotation2d(getHeading());
  }

  public void setHeading(double angle){
    gyro.setYaw(-angle);
  }

  public Pose2d getPose(){
    return odometer.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose){
    odometer.resetPosition(getRotation2d(), getModulePositions(), pose);
  }

  public SwerveModulePosition[] getModulePositions(){
    SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
    modulePositions[0] = frontLeft.getPosition();
    modulePositions[1] = frontRight.getPosition();
    modulePositions[2] = backLeft.getPosition();
    modulePositions[3] = backRight.getPosition();

    return modulePositions;
  }

  public void stop(){
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }

  public void setModuleStates(SwerveModuleState[] desiredState){
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredState, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);

    frontLeft.setDesiredState(desiredState[0]);
    frontRight.setDesiredState(desiredState[1]);
    backLeft.setDesiredState(desiredState[2]);
    backRight.setDesiredState(desiredState[3]);
  }

  public void resetStates(){
    frontLeft.resetState();
    frontRight.resetState();
    backLeft.resetState();
    backRight.resetState();
  }

  public ChassisSpeeds getChassisSpeeds(){
    return DriveConstants.kDriveKinematics.toChassisSpeeds(
      frontLeft.getSwerveModuleState(),
      frontRight.getSwerveModuleState(),
      backLeft.getSwerveModuleState(),
      backRight.getSwerveModuleState()
    );
  }


}
