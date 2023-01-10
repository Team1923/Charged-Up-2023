// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

  public static final double CAMERA_HEIGHT_METERS = 0;
  public static final double TARGET_HEIGHT_METERS = 0;
  public static final double CAMERA_PITCH_RADIANS = 0;

  public final class ArmConstants{
    public static final int shoulderMotorID = 0; //needs to be changed
    public static final int elbowMotorID = 0; //needs to be changed

    public static final double shoulderkP = 0;
    public static final double shoulderkI = 0;
    public static final double shoulderkD = 0;

    public static final double elbowkP = 0;
    public static final double elbowkI = 0;
    public static final double elbowkD = 0;

    public static final double shoulderkMaxVelocityRadPerSecond = 0;
    public static final double shoulderkMaxAccelerationRadPerSecSquared = 0;

    public static final double elbowkMaxVelocityRadPerSecond = 0;
    public static final double elbowkMaxAccelerationRadPerSecSquared = 0;


    //CCW positive
    public static final double kShoulderOffsetRads = Math.PI/2;
    public static final double kElbowOffsetRads = -Math.PI/2;

    public static final double shoulderGearRatio = 2;
    public static final double shoulderTicksToRad = (2 * Math.PI) / (ticksPerRev * shoulderGearRatio);
    public static final double shoulderRadsToTicks = 1 / shoulderTicksToRad;

    public static final double elbowGearRatio = 2;
    public static final double elbowTicksToRad = (2 * Math.PI) / (ticksPerRev * elbowGearRatio);
    public static final double elbowRadsToTicks = 1 / shoulderTicksToRad;

    public static final double shoulderMass = 1;
    public static final double elbowMass = 1;

    public static final double shoulderCGDistance = 1;
    public static final double elbowCGDistance = 1;

    public static final double lengthOfShoulder = 1;
    public static final double lengthOfElbow = 1;

    public static final double maxShoulderVel = 10000;
    public static final double maxShoulderAccel = 5000;

    public static final double maxElbowVel = 10000;
    public static final double maxElbowAccel = 5000;

    public static final double maxElbowGravityConstant = 1;
    public static final double maxShoulderGravityConstant = 1;

  }


    //falcon constants
    public static final double ticksPerRev = 2048;

    public static final int timeoutMs = 20;

    public static final double falconMaxRPM = 6380;

}

