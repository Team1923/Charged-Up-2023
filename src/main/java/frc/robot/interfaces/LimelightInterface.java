// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.interfaces;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimeLightConstants;

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
  private double limelight_height = LimeLightConstants.CAMERA_HEIGHT_METERS;
  // height in inches of center of target from ground
  private double target_height = LimeLightConstants.TARGET_HEIGHT_METERS;
  // limelight mounting angle above positive x axis in degrees
  private double limelight_mount_angle = LimeLightConstants.CAMERA_PITCH_RADIANS;

  public enum Limelight{
    LEFT_LIMELIGHT,
    RIGHT_LIMELIGHT
  }

  private static NetworkTable leftLimelight = NetworkTableInstance.getDefault().getTable("leftLimelight");
  private static NetworkTable rightLimelight = NetworkTableInstance.getDefault().getTable("rightLimelight");
  
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
    leftLimelight = NetworkTableInstance.getDefault().getTable("leftLimelight");
    rightLimelight = NetworkTableInstance.getDefault().getTable("rightLimelight");
  }

  public boolean validTargets(Limelight limelight) {
    if (getEntry("tv", limelight ) == 1.0) {
      return true;
    } else {
      return false;
    }
  }

  public double getHorizontalOffset(Limelight limelight) { /** LL1: -27 degrees to 27 degrees */
    return getEntry("tx", limelight);
  }

  public double getVerticalOffset(Limelight limelight) { /** LL1: -20.5 degrees to 20.5 degrees */
    return getEntry("ty", limelight);
  }

  public double getArea(Limelight limelight) { /** 0% of image to 100% of image */
    return getEntry("ta", limelight);
  }

  public double getSkew(Limelight limelight) { /** -90 degrees to 0 degrees */
    return getEntry("ts", limelight);
  }

  public double getLatency(Limelight limelight) { /** The pipeline’s latency contribution (ms) Add at least 11ms for image capture latency. */
    return getEntry("tl", limelight);
  }

  public double getColor(Limelight limelight) {
    return getEntry("tc", limelight );
  }

  public double getBoundingBoxShortLength(Limelight limelight) { /** Sidelength of shortest side of the fitted bounding box (pixels) */
    return getEntry("tshort", limelight);
  }

  public double getBoundingBoxLongLength(Limelight limelight) { /** Sidelength of longest side of the fitted bounding box (pixels) */
    return getEntry("tlong", limelight);
  }

  public double thor(Limelight limelight) { /** Horizontal sidelength of the rough bounding box (0 - 320 pixels) */
    return getEntry("tshort", limelight);
  }

  public double tvert(Limelight limelight) { /** Vertical sidelength of the rough bounding box (0 - 320 pixels) */
    return getEntry("tlong", limelight);
  }

  public double getPipeline(Limelight limelight) { /** True active pipeline index of the camera (0 .. 9) */
    return getEntry("getpipe", limelight);
  }

  public double getID(Limelight limelight){ // returns id of primary april tag
    return getEntry("tid", limelight);
  }

  public double getClassID(Limelight limelight){ // returns class id of primary neural detector result
    return getEntry("tclass", limelight);
  }

  public double[] getCamtransformation(Limelight limelight){ // returns translational and rotaional transformation of camera
    return getEntryArray("camtran", limelight);
  }

  public double[] getBotPose(Limelight limelight){ // does not work as of yet, returns pose as an array
    return getEntryArray("botpose", limelight);
  }



  public void setLEDMode(ledMode mode, Limelight limelight) {
    setEntry("ledMode", mode.state, limelight);
  }

  public double getLEDMode(Limelight limelight) {
    return getEntry("ledMode", limelight);
  }

  public boolean checkLimelight(Limelight limelight) {
    if(getLEDMode(limelight) == 1) {
      return true;
    } else {
      return false;
    }
  }

  public void setCamMode(camMode mode, Limelight limelight) {
    setEntry("camMode", mode.state, limelight);
  }

  public void setPipeline(int pipeline, Limelight limelight) { /** True active pipeline index of the camera (0 .. 9) */
    setEntry("pipeline", pipeline, limelight);
  }

  public void setStreamingMode(streamingMode mode, Limelight limelight) { /** True active pipeline index of the camera (0 .. 9) */
    setEntry("pipeline", mode.state, limelight);
  }

  public void setSnapshots(boolean snapshot, Limelight limelight) {
    if (snapshot) {
      setEntry("snapshot", 1, limelight);
    } else {
      setEntry("snapshot", 0, limelight);
    }
  } 

  private static void setEntry(String key, int value, Limelight limelight) {
    if(limelight == Limelight.LEFT_LIMELIGHT){
      leftLimelight.getEntry(key).setDouble(value);
    }
    else{
      rightLimelight.getEntry(key).setDouble(value);

    }
    
  }
  
  private static double getEntry(String key, Limelight limelight) {
    if(limelight == Limelight.LEFT_LIMELIGHT){
      return leftLimelight.getEntry(key).getDouble(0);
    }
    else{
      return rightLimelight.getEntry(key).getDouble(0);
    }
    
  }

  private static double[] getEntryArray(String key, Limelight limelight) {
    if(limelight == Limelight.LEFT_LIMELIGHT){
      return leftLimelight.getEntry(key).getDoubleArray(arr);
    }
    else{
      return rightLimelight.getEntry(key).getDoubleArray(arr);
    }

  }

  

  

  public double distanceToTarget(Limelight limelight) {
    // Returns distance to target assuming 
    return (target_height - limelight_height) /(Math.tan(Math.toRadians(limelight_mount_angle + (getVerticalOffset(limelight)))));


  }

  public Pose3d getRobotPose(Limelight limelight){
    double[] result  = getBotPose(limelight);
    Translation3d tran3d = new Translation3d(result[0], result [1], result[2]);
    Rotation3d r3d = new Rotation3d(result[3], result[4], result[5]);
    Pose3d p3d = new Pose3d(tran3d, r3d);

    return p3d;
  }

}