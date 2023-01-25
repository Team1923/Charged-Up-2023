// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.util.COTSFalconSwerveConstants;
import frc.robot.util.SwerveModuleConstants;

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

  public static final double stickDeadband = 0.1;


  public static final class Swerve {
      public static final int pigeonID = 12;
      public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-


      public static final COTSFalconSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot
          COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L3);


      /* Drivetrain Constants */
      public static final double trackWidth = Units.inchesToMeters(18.75); //TODO: This must be tuned to specific robot
      public static final double wheelBase = Units.inchesToMeters(18.75); //TODO: This must be tuned to specific robot
      public static final double wheelCircumference = chosenModule.wheelCircumference;


      /* Swerve Kinematics
       * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
       public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
          new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
          new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
          new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
          new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));


      /* Module Gear Ratios */
      public static final double driveGearRatio = chosenModule.driveGearRatio;
      public static final double angleGearRatio = chosenModule.angleGearRatio;


      /* Motor Inverts */
      public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
      public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;


      /* Angle Encoder Invert */
      public static final boolean canCoderInvert = chosenModule.canCoderInvert;


      /* Swerve Current Limiting */
      public static final int angleContinuousCurrentLimit = 25;
      public static final int anglePeakCurrentLimit = 40;
      public static final double anglePeakCurrentDuration = 0.1;
      public static final boolean angleEnableCurrentLimit = true;


      public static final int driveContinuousCurrentLimit = 35;
      public static final int drivePeakCurrentLimit = 60;
      public static final double drivePeakCurrentDuration = 0.1;
      public static final boolean driveEnableCurrentLimit = true;


      /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
       * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
      public static final double openLoopRamp = 0.25;
      public static final double closedLoopRamp = 0.0;


      /* Angle Motor PID Values */
      public static final double angleKP = chosenModule.angleKP;
      public static final double angleKI = chosenModule.angleKI;
      public static final double angleKD = chosenModule.angleKD;
      public static final double angleKF = chosenModule.angleKF;


      /* Drive Motor PID Values */
      public static final double driveKP = 0.05; //TODO: This must be tuned to specific robot
      public static final double driveKI = 0.0;
      public static final double driveKD = 0.0;
      public static final double driveKF = 0.0;


      /* Drive Motor Characterization Values
       * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
      public static final double driveKS = (0.32 / 12); //TODO: This must be tuned to specific robot
      public static final double driveKV = (1.51 / 12);
      public static final double driveKA = (0.27 / 12);


      /* Swerve Profiling Values */
      /** Meters per Second */
      public static final double maxSpeed = 5.4864; //TODO: This must be tuned to specific robot
      /** Radians per Second */
      public static final double maxAngularVelocity = 54; //TODO: This must be tuned to specific robot


      /* Neutral Modes */
      public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
      public static final NeutralMode driveNeutralMode = NeutralMode.Brake;


      /* Module Specific Constants */
      /* Front Left Module - Module 0 */
      public static final class Mod0 { //TODO: This must be tuned to specific robot
          public static final int driveMotorID = 0;
          public static final int angleMotorID = 1;
          public static final int canCoderID = 8;
          public static final Rotation2d angleOffset = Rotation2d.fromDegrees(29.8);
          public static final SwerveModuleConstants constants =
              new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
      }


      /* Front Right Module - Module 1 */
      public static final class Mod1 { //TODO: This must be tuned to specific robot
          public static final int driveMotorID = 2;
          public static final int angleMotorID = 3;
          public static final int canCoderID = 9;
          public static final Rotation2d angleOffset = Rotation2d.fromDegrees(4.75);
          public static final SwerveModuleConstants constants =
              new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
      }
     
      /* Back Left Module - Module 2 */
      public static final class Mod2 { //TODO: This must be tuned to specific robot
          public static final int driveMotorID = 4;
          public static final int angleMotorID = 5;
          public static final int canCoderID = 10;
          public static final Rotation2d angleOffset = Rotation2d.fromDegrees(350.59);
          public static final SwerveModuleConstants constants =
              new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
      }


      /* Back Right Module - Module 3 */
      public static final class Mod3 { //TODO: This must be tuned to specific robot
          public static final int driveMotorID = 6;
          public static final int angleMotorID = 7;
          public static final int canCoderID = 11;
          public static final Rotation2d angleOffset = Rotation2d.fromDegrees(93.6);
          public static final SwerveModuleConstants constants =
              new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
      }
  }


  public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
      public static final double kMaxSpeedMetersPerSecond = 3;
      public static final double kMaxAccelerationMetersPerSecondSquared = 3;
      public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
      public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
 
      public static final double kPXController = 1;
      public static final double kPYController = 1;
      public static final double kPThetaController = 1;
 
      /* Constraint for the motion profilied robot angle controller */
      public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
          new TrapezoidProfile.Constraints(
              kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
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

