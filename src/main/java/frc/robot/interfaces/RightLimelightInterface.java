// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.interfaces;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimeLightConstants;

/**
 * This class allows for interaction with the Limelight
 * via a NetworkTable
 */
public class RightLimelightInterface extends SubsystemBase {

  private static RightLimelightInterface limelight;

  // height in inches of camera from ground
  private double limelight_height = LimeLightConstants.CAMERA_HEIGHT_METERS;
  // height in inches of center of target from ground
  private double target_height = LimeLightConstants.TARGET_HEIGHT_METERS;
  // limelight mounting angle above positive x axis in degrees
  private double limelight_mount_angle = LimeLightConstants.CAMERA_PITCH_RADIANS;

  private static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-left");
  
  public static boolean tracking = false;

  public static synchronized RightLimelightInterface getInstance() {
    if (limelight == null) {
      limelight = new RightLimelightInterface();
    }
    return limelight;
  }

  public enum ledMode {
    USE_PIPELINE(0),
    OFF(1),
    BLINK(2),
    ON(3);
    public final int state;
    private ledMode(int mode) {
      this.state = mode;
    }
  }

  public enum camMode {
    VISION_PROCESSOR(0),
    DRIVER_CAMERA(1);
    public final int state;
    private camMode(int mode) {
      this.state = mode;
    }
  }

  public enum streamingMode {
    STANDARD(0),
    PiP_MAIN(1),
    PiP_SECONDARY(2);
    public final int state;
    private streamingMode(int mode) {
      this.state = mode;
    }
  }

  public RightLimelightInterface() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    table = NetworkTableInstance.getDefault().getTable("limelight-left");
  }

  public boolean validTargets() {
    if (getEntry("tv") == 1.0) {
      return true;
    } else {
      return false;
    }
  }

  public double getHorizontalOffset() { /** LL1: -27 degrees to 27 degrees */
    return getEntry("tx");
  }

  public double getVerticalOffset() { /** LL1: -20.5 degrees to 20.5 degrees */
    return getEntry("ty");
  }

  public double getArea() { /** 0% of image to 100% of image */
    return getEntry("ta");
  }

  public double getSkew() { /** -90 degrees to 0 degrees */
    return getEntry("ts");
  }

  public double getLatency() { /** The pipeline’s latency contribution (ms) Add at least 11ms for image capture latency. */
    return getEntry("tl");
  }

  public double getColor() {
    return getEntry("tc");
  }

  public double getBoundingBoxShortLength() { /** Sidelength of shortest side of the fitted bounding box (pixels) */
    return getEntry("tshort");
  }

  public double getBoundingBoxLongLength() { /** Sidelength of longest side of the fitted bounding box (pixels) */
    return getEntry("tlong");
  }

  public double thor() { /** Horizontal sidelength of the rough bounding box (0 - 320 pixels) */
    return getEntry("tshort");
  }

  public double tvert() { /** Vertical sidelength of the rough bounding box (0 - 320 pixels) */
    return getEntry("tlong");
  }

  public double getPipeline() { /** True active pipeline index of the camera (0 .. 9) */
    return getEntry("getpipe");
  }

  public void setLEDMode(ledMode mode) {
    setEntry("ledMode", mode.state);
  }

  public double getLEDMode() {
    return getEntry("ledMode");
  }

  public boolean checkLimelight() {
    if(getLEDMode() == 1) {
      return true;
    } else {
      return false;
    }
  }

  public void setCamMode(camMode mode) {
    setEntry("camMode", mode.state);
  }

  public void setPipeline(int pipeline) { /** True active pipeline index of the camera (0 .. 9) */
    setEntry("pipeline", pipeline);
  }

  public void setStreamingMode(streamingMode mode) { /** True active pipeline index of the camera (0 .. 9) */
    setEntry("pipeline", mode.state);
  }

  public void setSnapshots(boolean snapshot) {
    if (snapshot) {
      setEntry("snapshot", 1);
    } else {
      setEntry("snapshot", 0);
    }
  } 

  private static void setEntry(String key, int value) {
    table.getEntry(key).setDouble(value);
  }
  
  private static double getEntry(String key) {
    return table.getEntry(key).getDouble(0);
  }

  public double distanceToTarget() {
    // Returns distance to target assuming 
    return (target_height - limelight_height) / (Math.tan(Math.toRadians(limelight_mount_angle + getVerticalOffset())));
  }

}