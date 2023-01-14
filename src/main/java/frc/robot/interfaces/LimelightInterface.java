// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.interfaces;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This class allows for interaction with the Limelight
 * via a NetworkTable
 */
public class LimelightInterface extends SubsystemBase {

  private static LimelightInterface limelight;

  private static double[] arr = {999,999,999};

  public static synchronized LimelightInterface getInstance() {
    if (limelight == null) {
      limelight = new LimelightInterface();
    }
    return limelight;
  }

  /*
   * TO-DO: add limelight constants
   */
  // height in inches of camera from ground
  private double limelight_height = 21.5;
  // height in inches of center of target from ground
  private double target_height = 29;
  // limelight mounting angle above positive x axis in degrees
  private double limelight_mount_angle = -4;

  private static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  
  public static boolean tracking = false;

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

  public LimelightInterface() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    table = NetworkTableInstance.getDefault().getTable("limelight");
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

  public double getID(){ // returns id of primary april tag
    return getEntry("tid");
  }

  public double getClassID(){ // returns class id of primary neural detector result
    return getEntry("tclass");
  }

  public double[] getCamtransformation(){ // returns translational and rotaional transformation of camera
    return getEntryArray("camtran");
  }

  public double[] getBotPose(){ // does not work as of yet, returns pose as an array
    return getEntryArray("botpose");
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

  private static double[] getEntryArray(String key) {
    return table.getEntry(key).getDoubleArray(arr);
  }

  

  

  public double distanceToTarget() {
    // Returns distance to target assuming 
    return (target_height - limelight_height) /(Math.tan(Math.toRadians(limelight_mount_angle + (getVerticalOffset()))));


  }

  public Pose3d getRobotPose(){
    double[] result  = getBotPose();
    Translation3d tran3d = new Translation3d(result[0], result [1], result[2]);
    Rotation3d r3d = new Rotation3d(result[3], result[4], result[5]);
    Pose3d p3d = new Pose3d(tran3d, r3d);

    return p3d;
  }

}