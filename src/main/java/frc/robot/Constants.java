// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode currentMode = Mode.REAL;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }


  public final class LimeLightConstants{
    public static final double CAMERA_HEIGHT_METERS = 0;
    public static final double TARGET_HEIGHT_METERS = 0;
    public static final double CAMERA_PITCH_RADIANS = 0;
  }

  
  public final class FalconConstants{
    public static final double ticksPerRev = 2048;
    public static final int timeoutMs = 20;
    public static final double falconMaxRPM = 6380;
  }

  public final class ArmConstants{
    public static final int shoulderMotorID = 18; //needs to be changed
    public static final int elbowMotorID = 17; //needs to be changed

    public static final double shoulderkP = 0.1;
    public static final double shoulderkI = 0;
    public static final double shoulderkD = 0;

    public static final double elbowkP = 0.1;
    public static final double elbowkI = 0;
    public static final double elbowkD = 0;


    //CCW positive
    public static final double kShoulderOffsetRads = 0;
    public static final double kElbowOffsetRads = 0;

    public static final double shoulderGearRatio = 100 * 16 / 16;
    public static final double shoulderTicksToRad = (2 * Math.PI) / (FalconConstants.ticksPerRev * shoulderGearRatio);
    public static final double shoulderRadsToTicks = 1 / shoulderTicksToRad;

    public static final double elbowGearRatio = 100 * 16 / 16;
    public static final double elbowTicksToRad = (2 * Math.PI) / (FalconConstants.ticksPerRev * elbowGearRatio);
    public static final double elbowRadsToTicks = 1 / shoulderTicksToRad;

    public static final double shoulderMass = 2;
    public static final double elbowMass = 0.454;

    public static final double shoulderCGDistance = 0.279;
    public static final double elbowCGDistance = 0.305;

    public static final double lengthOfShoulder = 0.889;
    public static final double lengthOfElbow = 0.61;

    public static final double maxShoulderVel = 35000;
    public static final double maxShoulderAccel = 20000;

    public static final double maxElbowVel = 2*35000;
    public static final double maxElbowAccel = 4*20000;

    public static final double maxElbowGravityConstant = 0;
    public static final double maxShoulderGravityConstant = 0.06;

    public static final double elbowHome = -Math.PI/2;
    public static final double shoulderHome = Math.PI/2;

  }

  public static final class ModuleConstants {
    public static final double kTicksPerRotation = 2048;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4); 
    public static final double kDriveMotorGearRatio = 1.0 / 6.12;                
    public static final double kTurningGearRatio = 1.0 / 21.428;                   
    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kDriveEncoderTicks2Meter = kDriveEncoderRot2Meter / kTicksPerRotation;  
    public static final double kTurningEncoderRot2Rad = kTurningGearRatio * 2 * Math.PI;
    public static final double kturningEncoderTicks2Rad = kTurningEncoderRot2Rad / kTicksPerRotation; 
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60; 
    public static final double kDriveEncoderTicks2MeterPerSec = (10 * 60 / kTicksPerRotation) * kDriveEncoderRPM2MeterPerSec; 

    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
    public static final double kTurningEncoderTicks2RadPerSec = (10 * 60 / kTicksPerRotation) * kTurningEncoderRPM2RadPerSec; 
    public static final double kPTurning = 0.4;                                
    public static final double kDTurning = 0.0;
}

  public static final class DriveConstants {
    //ID for Pigeon2 - need to update
    public static final int pigeonCANID = 12;

    // Distance between right and left wheels
    public static final double kTrackWidth = Units.inchesToMeters(18.75); 
    // Distance between front and back wheels
    public static final double kWheelBase = Units.inchesToMeters(18.75); 

    /*the sign of the Translation2d objects were updated to match 1706
      For driving purposes, we can revert the constants if needed.
      Keep in mind that to work with PathPlanner, however,
      maintaining these directions are necessary. 
    */
    
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),   // + -
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),    // + +
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),  // - -
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));  // - +


    public static final double kPhysicalMaxSpeedMetersPerSecond = 5.4864;                   
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 54;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 4;

    // Front Left Module - UPDATE
    public static final int kFrontLeftDriveMotorPort = 0;
    public static final int kFrontLeftTurningMotorPort = 1;
    public static final boolean kFrontLeftDriveReversed = true;
    public static final boolean kFrontLeftTurningReversed = true;
    public static final int kFrontLeftDriveAbsoluteEncoderPort = 8;
    public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 5.74; //0.77
    public static final boolean kFrontLeftDriveAbsoluteEncoderOffsetReversed = false;

    // Front Right Module - UPDATE
    public static final int kFrontRightDriveMotorPort = 2;
    public static final int kFrontRightTurningMotorPort = 3;
    public static final boolean kFrontRightDriveReversed = true;
    public static final boolean kFrontRightTurningReversed = true;
    public static final int kFrontRightDriveAbsoluteEncoderPort = 9;
    public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = -0.2;  //0.56
    public static final boolean kFrontRightDriveAbsoluteEncoderOffsetReversed = false;

    // Back Right Module - UPDATE
    public static final int kBackRightDriveMotorPort = 4; 
    public static final int kBackRightTurningMotorPort = 5;
    public static final boolean kBackRightDriveReversed = false;
    public static final boolean kBackRightTurningReversed = true;
    public static final int kBackRightDriveAbsoluteEncoderPort = 10;
    public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 1.72; //0.4
    public static final boolean kBackRightDriveAbsoluteEncoderOffsetReversed = false;

    // Back Left Module - UPDATE
    public static final int kBackLeftDriveMotorPort = 6;
    public static final int kBackLeftTurningMotorPort = 7;
    public static final boolean kBackLeftDriveReversed = true;
    public static final boolean kBackLeftTurningReversed = true;
    public static final int kBackLeftDriveAbsoluteEncoderPort = 11;
    public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 0.2; //3.71
    public static final boolean kBackLeftDriveAbsoluteEncoderOffsetReversed = false;

    public static final double kMinRotationCommand = DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond
    * Math.pow(.1, 2);
    // Minimum allowable tranlsation command (in m/s) assuming user input is squared
    // using quadraticTransform, this value is always positive and should be
    // compared agaisnt the absolute value of the drive command
    public static final double kMinTranslationCommand = DriveConstants.kTeleDriveMaxSpeedMetersPerSecond
    * Math.pow(.1, 2);
}

 

}

