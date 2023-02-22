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

  public final class LimeLightConstants {
    public static final double CAMERA_HEIGHT_METERS = 0;
    public static final double TARGET_HEIGHT_METERS = 0;
    public static final double CAMERA_PITCH_RADIANS = 0;
  }

  public final class FalconConstants {
    public static final double ticksPerRev = 2048;
    public static final int timeoutMs = 20;
    public static final double falconMaxRPM = 6380;
  }

  public final class IntakeConstants {
    public static final int intakeProximalID = 16;
    public static final int intakeDistalID = 17;
    public static final int leftIntakeWheelMotor = 18;
    public static final int rightIntakeWheelMotor = 19;
    public static final int intakeProximalAbsoluteEncoderID = 2; //2
    public static final int intakeDistalAbsoluteEncoderID = 3; //3

    public static final double intakeProximalCGDistance = 0.0762;
    public static final double intakeDistalCGDistance = 0.1524;

    public static final double intakeProximalLength = 0.1397;
    public static final double intakeDistalLength = 0.2286;

    public static final double intakeProximalMass = 0.453592;
    public static final double intakeDistalMass = 3.17515;

    public static final double intakeProximalkP = 0.1;
    public static final double intakeProximalkI = 0;
    public static final double intakeProximalkD = 0;
    public static final double intakeDistalkP = 0.2;
    public static final double intakeDistalkI = 0;
    public static final double intakeDistalkD = 0;

    public static final double maxIntakeProximalVel = 30000;
    public static final double maxIntakeProximalAccel = 30000;
    public static final double maxIntakeDistalVel = 30000;
    public static final double maxIntakeDistalAccel = 30000;

    public static final double intakeProximalGearRatio = 64;
    public static final double intakeProximalTicksToRad = (2 * Math.PI)
        / (FalconConstants.ticksPerRev * intakeProximalGearRatio);
    public static final double intakeProximalRadsToTicks = 1 / intakeProximalTicksToRad;

    public static final double intakeDistalGearRatio = 64;
    public static final double intakeDistalTicksToRad = (2 * Math.PI)
        / (FalconConstants.ticksPerRev * intakeDistalGearRatio);
    public static final double intakeDistalRadsToTicks = 1 / intakeDistalTicksToRad;

    public static final double intakeProximalHardstop = 2.971;
    public static final double intakeDistalHardstop = 1.307;
    public static final double proximalEncoderZero = 0.522;
    public static final double distalEncoderZero = 4.455;

    public static final double intakeMaxProximalGravityConstant = 0.2;
    public static final double intakeMaxDistalGravityConstant = 0.15;

    public static final double intakeProximalAbsoluteEncoderToTicks = intakeProximalGearRatio * 2048;
    public static final double intakeDistalAbsoluteEncoderToTicks = intakeDistalGearRatio * 2048;
    public static final double intakeProximalAbsoluteEncoderToRadians = intakeProximalAbsoluteEncoderToTicks
        * intakeProximalTicksToRad;
    public static final double intakeDistalAbsoluteEncoderToRadians = intakeDistalAbsoluteEncoderToTicks
        * intakeDistalTicksToRad;

    public static final double kIntakeProximalOffsetRads = 0;
    public static final double kIntakeDistalOffsetRads = 0;

    public static final double errorThreshold = 0.1;
    public static final double cubeIntakeSpeed = .2;
    public static final double coneIntakeSpeed = .75;
    public static final double ejectSpeed = -1;
    public static final double cubeCurrentThreshold = 80;
    public static final double coneCurrentThreshold = 80;

    public static final double gripSpeed = .15;
    public static final double handoffSpeed = -.10;
  }

  public final class ArmConstants {
    public static final int proximalMotorID = 15; // needs to be changed
    public static final int distalMotorID = 14; // needs to be changed

    public static final double proximalkP = 0.2;
    public static final double proximalkI = 0.0001;
    public static final double proximalkD = 0;

    public static final double distalkP = 0.2;
    public static final double distalkI = 0.0001;
    public static final double distalkD = 0;

    // CCW positive
    public static final double kProximalOffsetRads = 0;
    public static final double kDistalOffsetRads = 0;

    public static final double proximalGearRatio = 180;
    public static final double proximalTicksToRad = (2 * Math.PI) / (FalconConstants.ticksPerRev * proximalGearRatio);
    public static final double proximalRadsToTicks = 1 / proximalTicksToRad;

    public static final double distalGearRatio = 180;
    public static final double distalTicksToRad = (2 * Math.PI) / (FalconConstants.ticksPerRev * distalGearRatio);
    public static final double distalRadsToTicks = 1 / proximalTicksToRad;

    public static final double proximalMass = 1.81437;
    public static final double distalMass = 0.907185;

    public static final double proximalCGDistance = 0.7112;
    public static final double distalCGDistance = 0.5715;

    public static final double lengthOfProximal = 1.0922;
    public static final double lengthOfDistal = 0.83185;

    public static final double maxProximalVel = 10000;
    public static final double maxProximalAccel = 10000;

    public static final double maxDistalVel = 20000;
    public static final double maxDistalAccel = 20000;

    public static final double maxDistalGravityConstant = 0;
    // THE NUMERICAL GRAVITY CONSTANT IS DIVIDED BY COS(21 degrees)
    public static final double maxProximalGravityConstant = 0.1968;

    public static final double proximalCobra = 110 * Math.PI / 180;
    public static final double distalCobra = 0;

    public static final int proximalEncoderID = 0;
    public static final int distalEncoderID = 1;

    public static final double proximalAbsoluteEncoderToTicks = ArmConstants.proximalGearRatio * 2048;
    public static final double distalAbsoluteEncoderToTicks = ArmConstants.distalGearRatio * 2048;
    public static final double proximalAbsoluteEncoderToRadians = ArmConstants.proximalAbsoluteEncoderToTicks
        * ArmConstants.proximalTicksToRad;
    public static final double distalAbsoluteEncoderToRadians = ArmConstants.distalAbsoluteEncoderToTicks
        * ArmConstants.distalTicksToRad;

    public static final double proximalHardstop = Math.PI/2;
    public static final double distalHardstop = -Math.PI/2;
    public static final double proximalEncoderZero = 0.460;
    public static final double distalEncoderZero = -0.821;

    public static final double errorThreshold = 0.05;

    public static final double minProximalPosition = Math.PI/4;
    public static final double maxProximalPosition = 3*Math.PI/4;

    public static final double maxDistalPosition = Math.PI/6;
    public static final double minDistalPosition = -7*Math.PI/6;

  }

  public final class GamePieceColorConstants {
    // public static final double CONEREDTOGREEN = 0.8175;
    // public static final double CONEBLUETOGREEN = 1.57;
    // public static final double CONEREDTOBLUE = 0.523;

    // public static final double CUBEREDTOGREEN = 1.085;
    // public static final double CUBEBLUETOGREEN = 1.945;
    // public static final double CUBEREDTOBLUE = 0.556;

    public static final double UNKNOWN_THRESHOLD = 1.8; // may need to be adjusted based on lighting environment
    public static final double Offset = 0.3;

  }

  public final class DigitalIDConstants {
    public static final int DigitalID = 0;// Change later
  }

  public static final double stickDeadband = 0.1;

  public static final class Swerve {
    public static final int pigeonID = 13;
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

    public static final COTSFalconSwerveConstants chosenModule = // TODO: This must be tuned to specific robot
        COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L3);

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(18.75); // TODO: This must be tuned to specific robot
    public static final double wheelBase = Units.inchesToMeters(18.75); // TODO: This must be tuned to specific robot
    public static final double wheelCircumference = chosenModule.wheelCircumference;

    /*
     * Swerve Kinematics No need to ever change this unless you are not doing a
     * traditional rectangular/square 4 module swerve
     */
    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2.0, trackWidth / 2.0), new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0), new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

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

    /*
     * These values are used by the drive falcon to ramp in open loop and closed
     * loop driving. We found a small open loop ramp (0.25) helps with tread wear,
     * tipping, etc
     */
    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    /* Angle Motor PID Values */
    public static final double angleKP = chosenModule.angleKP;
    public static final double angleKI = chosenModule.angleKI;
    public static final double angleKD = chosenModule.angleKD;
    public static final double angleKF = chosenModule.angleKF;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.05; // TODO: This must be tuned to specific robot
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKF = 0.0;

    /*
     * Drive Motor Characterization Values Divide SYSID values by 12 to convert from
     * volts to percent output for CTRE
     */
    public static final double driveKS = (0.32 / 12); // TODO: This must be tuned to specific robot
    public static final double driveKV = (1.51 / 12);
    public static final double driveKA = (0.27 / 12);

    /* Swerve Profiling Values */
    /** Meters per Second */
    public static final double maxSpeed = 5.4864; // TODO: This must be tuned to specific robot
    /** Radians per Second */
    public static final double maxAngularVelocity = 54; // TODO: This must be tuned to specific robot

    /* Neutral Modes */
    public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
    public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 { // TODO: This must be tuned to specific robot
      public static final int driveMotorID = 1;
      public static final int angleMotorID = 2;
      public static final int canCoderID = 9;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(29.8);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 { // TODO: This must be tuned to specific robot
      public static final int driveMotorID = 3;
      public static final int angleMotorID = 4;
      public static final int canCoderID = 10;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(4.75);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 { // TODO: This must be tuned to specific robot
      public static final int driveMotorID = 5;
      public static final int angleMotorID = 6;
      public static final int canCoderID = 11;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(350.59);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 { // TODO: This must be tuned to specific robot
      public static final int driveMotorID = 7;
      public static final int angleMotorID = 8;
      public static final int canCoderID = 12;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(93.6);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }
  }

  public static final class AutoConstants { // TODO: The below constants are used in the example auto, and must be tuned
                                            // to specific robot
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    public static final double looperUpdateTime = 0.01;

    /* Constraint for the motion profilied robot angle controller */
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

}
