// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  private WPI_TalonFX shoulderMotor = new WPI_TalonFX(ArmConstants.shoulderMotorID);
  private WPI_TalonFX elbowMotor = new WPI_TalonFX(ArmConstants.elbowMotorID);

  private double goalShoulderPosition;
  private double goalElbowPosition;
  
  public ArmSubsystem() {
    shoulderMotor.configFactoryDefault();
    elbowMotor.configFactoryDefault();
    
    shoulderMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.timeoutMs);
    elbowMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.timeoutMs);

    shoulderMotor.config_kP(0, ArmConstants.shoulderkP, Constants.timeoutMs);
    shoulderMotor.config_kI(0, ArmConstants.shoulderkI, Constants.timeoutMs);
    shoulderMotor.config_kD(0, ArmConstants.shoulderkD, Constants.timeoutMs);

    elbowMotor.config_kP(0, ArmConstants.elbowkP, Constants.timeoutMs);
    elbowMotor.config_kI(0, ArmConstants.elbowkI, Constants.timeoutMs);
    elbowMotor.config_kD(0, ArmConstants.elbowkD, Constants.timeoutMs);

    resetShoulderPosition(Math.PI/2);
    resetElbowPosition(-Math.PI/2);

    goalShoulderPosition = ArmConstants.shoulderHome;
    goalElbowPosition = ArmConstants.elbowHome;

    shoulderMotor.configMotionCruiseVelocity(ArmConstants.maxShoulderVel);
    shoulderMotor.configMotionAcceleration(ArmConstants.maxShoulderAccel);
    elbowMotor.configMotionCruiseVelocity(ArmConstants.maxElbowVel);
    elbowMotor.configMotionAcceleration(ArmConstants.maxElbowAccel);

    shoulderMotor.setNeutralMode(NeutralMode.Coast);
    elbowMotor.setNeutralMode(NeutralMode.Coast);

  }

  public void resetShoulderPosition(double setPoint){
    shoulderMotor.setSelectedSensorPosition(setPoint * ArmConstants.shoulderRadsToTicks);
  }

  public void resetElbowPosition(double setPoint){
    elbowMotor.setSelectedSensorPosition(-setPoint * ArmConstants.shoulderRadsToTicks);
  }

  public void resetShoulderEncoders(){
    shoulderMotor.setSelectedSensorPosition(0);
  }

  public void resetElbowEncoders(){
    elbowMotor.setSelectedSensorPosition(0);
  }

  // public void setShoulderPosition(double setPoint){
  //   shoulderMotor.set(ControlMode.MotionMagic, setPoint, DemandType.ArbitraryFeedForward, calculateShoulderFeedforward());
  // }

  // public void setElbowPosition(double setPoint){
  //   elbowMotor.set(ControlMode.MotionMagic, -setPoint, DemandType.ArbitraryFeedForward, calculateElbowFeedforward());
  // }

  public void setShoulderPosition(){
    shoulderMotor.set(ControlMode.MotionMagic, getShoulderGoal() * ArmConstants.shoulderRadsToTicks, DemandType.ArbitraryFeedForward, calculateShoulderFeedforward());
  }

  public void setElbowPosition(){
    elbowMotor.set(ControlMode.MotionMagic, ((-getElbowGoal()-((Math.PI/2)-getShoulderGoal())) * ArmConstants.elbowRadsToTicks), DemandType.ArbitraryFeedForward, calculateElbowFeedforward());
  }

  // public void setArmCartesian(double x, double y){
  //   double elbowGoal = (Math.acos(((x*x) + (y*y) - (Math.pow(ArmConstants.lengthOfShoulder, 2)) - (Math.pow(ArmConstants.lengthOfElbow, 2))) 
  //   / (2 * ArmConstants.lengthOfShoulder * ArmConstants.lengthOfElbow)));

  //   double shoulderGoal = (Math.atan(y/x) - 
  //     Math.atan((ArmConstants.lengthOfElbow * Math.sin(elbowGoal)) / (ArmConstants.lengthOfShoulder + ArmConstants.lengthOfElbow * Math.cos(elbowGoal))));

  //   shoulderGoal = (Math.PI/2 - shoulderGoal);
  //   setShoulderGoal(shoulderGoal);

  //   double newElbowGoal = Math.atan((y - ArmConstants.lengthOfShoulder*Math.sin(shoulderGoal)) / 
  //     (x - ArmConstants.lengthOfShoulder*Math.cos(shoulderGoal)));
  //   System.out.println("Shoulder Goal: " + shoulderGoal);
  //   setElbowGoal(newElbowGoal);
  // }

  public void setArmCartesianOld(double x, double y){
    double elbowGoal = (Math.acos(((x*x) + (y*y) - (Math.pow(ArmConstants.lengthOfShoulder, 2)) - (Math.pow(ArmConstants.lengthOfElbow, 2))) 
    / (2 * ArmConstants.lengthOfShoulder * ArmConstants.lengthOfElbow)));

    double shoulderGoal = (Math.atan(y/x) - 
      Math.atan((ArmConstants.lengthOfElbow * Math.sin(elbowGoal)) / (ArmConstants.lengthOfShoulder + ArmConstants.lengthOfElbow * Math.cos(elbowGoal))));

    setShoulderGoal(shoulderGoal);
    setElbowGoal(elbowGoal +shoulderGoal);
  }

  public double[] calculateArmCartesian(double x, double y){
    double elbowGoal = (Math.acos(((x*x) + (y*y) - (Math.pow(ArmConstants.lengthOfShoulder, 2)) - (Math.pow(ArmConstants.lengthOfElbow, 2))) 
    / (2 * ArmConstants.lengthOfShoulder * ArmConstants.lengthOfElbow)));

    double shoulderGoal = (Math.atan(y/x) - 
      Math.atan((ArmConstants.lengthOfElbow * Math.sin(elbowGoal)) / (ArmConstants.lengthOfShoulder + ArmConstants.lengthOfElbow * Math.cos(elbowGoal))));

    double[] conv = {shoulderGoal, elbowGoal};

    return conv;
  }

  public double[] calculateCircleIntersection(double x1, double y1, double r1, double x2, double y2, double r2){
    double[] solutions = new double[2];
    double centerdx = x1 - x2;
    double centerdy = y1 - y2;
    double R = Math.sqrt(centerdx * centerdx + centerdy * centerdy);
    if (!(Math.abs(r1 - r2) <= R && R <= r1 + r2)) { // no intersection
      setShoulderGoal(ArmConstants.shoulderHome);
      setElbowGoal(ArmConstants.elbowHome);
      return null;
    }
    // intersection(s) should exist

    double R2 = R*R;
    double R4 = R2*R2;
    double a = (r1*r1 - r2*r2) / (2 * R2);
    double r2r2 = (r1*r1 - r2*r2);
    double c = Math.sqrt(2 * (r1*r1 + r2*r2) / R2 - (r2r2 * r2r2) / R4 - 1);

    double fx = (x1+x2) / 2 + a * (x2 - x1);
    double gx = c * (y2 - y1) / 2;
    double ix1 = fx + gx;
    double ix2 = fx - gx;

    double fy = (y1+y2) / 2 + a * (y2 - y1);
    double gy = c * (x1 - x2) / 2;
    double iy1 = fy + gy;
    double iy2 = fy - gy;

    // note if gy == 0 and gx == 0 then the circles are tangent and there is only one solution
    // but that one solution will just be duplicated as the code is currently written
    if(iy2 >= iy1){
      solutions[0] = ix2;
      solutions[1] = iy2;
    }
    else{
      solutions[0] = ix1;
      solutions[1] = iy1;
    }

    return solutions;
  }

  public void setArmCartesian(double x, double y){
    double[] intersection = calculateCircleIntersection(0, 0, ArmConstants.lengthOfShoulder, x, y, ArmConstants.lengthOfElbow);
    double shoulderAngle = ArmConstants.shoulderHome; //setting these as default
    double elbowAngle = ArmConstants.elbowHome; //setting these as default
    if(intersection[0] < 0){
      shoulderAngle = Math.PI - Math.asin(intersection[1] / ArmConstants.lengthOfShoulder);
      elbowAngle = Math.PI - Math.asin((y-intersection[1]) / ArmConstants.lengthOfElbow);
    }
    else{
      shoulderAngle = Math.asin(intersection[1] / ArmConstants.lengthOfShoulder);
      elbowAngle = Math.asin((y-intersection[1]) / ArmConstants.lengthOfElbow);
    }
    setShoulderGoal(shoulderAngle);
    setElbowGoal(elbowAngle);
  }

  public double getShoulderPosition(){
    return (shoulderMotor.getSelectedSensorPosition() * ArmConstants.shoulderTicksToRad) + ArmConstants.kShoulderOffsetRads;
  }

  public double getElbowPosition(){
    return (-elbowMotor.getSelectedSensorPosition() * ArmConstants.elbowTicksToRad) + ArmConstants.kElbowOffsetRads;
  }


  public double getShoulderGoal(){
    return goalShoulderPosition;
  }

  public double getElbowGoal(){
    return goalElbowPosition;
  }

  public void setShoulderGoal(double sGoal){
    goalShoulderPosition = sGoal;
  }

  public void setElbowGoal(double eGoal){
    goalElbowPosition = eGoal;
  }

  public void setShoulderVoltage(double stpt){
    shoulderMotor.setVoltage(stpt);
  }

  public void setElbowVoltage(double stpt){
    elbowMotor.setVoltage(stpt);
  }

  public void stop(){
    shoulderMotor.stopMotor();
    elbowMotor.stopMotor();
  }

  public double getAngleToCG(){
    double shoulderCGX = Math.cos(getShoulderPosition()) * ArmConstants.shoulderCGDistance;
    double shoulderCGY = Math.sin(getShoulderPosition()) * ArmConstants.shoulderCGDistance;
    double shoulderLengthX = Math.cos(getShoulderPosition()) * ArmConstants.lengthOfShoulder;
    double shoulderLengthY = Math.sin(getShoulderPosition()) * ArmConstants.lengthOfShoulder;

    double elbowCGX = Math.cos(getElbowPosition()) * ArmConstants.elbowCGDistance + shoulderLengthX;
    double elbowCGY = Math.sin(getElbowPosition()) * ArmConstants.elbowCGDistance + shoulderLengthY;

    double averageXG = ((ArmConstants.elbowMass * elbowCGX) + ArmConstants.shoulderMass * shoulderCGX) / (ArmConstants.elbowMass + ArmConstants.shoulderMass);
    double averageYG = ((ArmConstants.elbowMass * elbowCGY) + ArmConstants.shoulderMass * shoulderCGY) / (ArmConstants.elbowMass + ArmConstants.shoulderMass);

    return Math.atan(averageYG / averageXG);
  }

  public double calculateShoulderFeedforward(){
    return ArmConstants.maxShoulderGravityConstant * Math.cos(getAngleToCG());
  }

  public double calculateElbowFeedforward(){
    return ArmConstants.maxElbowGravityConstant * Math.cos(getElbowPosition());
  }

  public double[] anglesToCartesian(double shoulderTheta, double elbowTheta) {

    double x = ArmConstants.lengthOfShoulder*Math.cos(shoulderTheta) + ArmConstants.lengthOfElbow*Math.cos(shoulderTheta+elbowTheta);
    double y = ArmConstants.lengthOfShoulder*Math.sin(shoulderTheta) + ArmConstants.lengthOfElbow*Math.sin(shoulderTheta+elbowTheta);

    double[] conv = {x,y};
    return conv;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elbow Position:", Math.toDegrees(getElbowPosition()));
    SmartDashboard.putNumber("Shoulder Position:", Math.toDegrees(getShoulderPosition()));

    SmartDashboard.putNumber("CG Angle:", Math.toDegrees(getAngleToCG()));
    
    SmartDashboard.putNumber("Elbow Goal:", getElbowGoal());
    SmartDashboard.putNumber("Shoulder Goal:", getShoulderGoal());

    SmartDashboard.putNumber("Elbow Output", elbowMotor.getMotorOutputPercent());

    double[] angles = calculateArmCartesian(1, 1);

    SmartDashboard.putNumber("Polar Shoulder: ", Math.toDegrees(angles[0]));
    SmartDashboard.putNumber("Polar Elbow: ", Math.toDegrees(angles[1]));

    double[] cartesian = anglesToCartesian(angles[0],angles[1]);

    SmartDashboard.putNumber("Cartesian Shoulder: ", cartesian[0]);
    SmartDashboard.putNumber("Cartesian Elbow: ", cartesian[1]);

  }
}
