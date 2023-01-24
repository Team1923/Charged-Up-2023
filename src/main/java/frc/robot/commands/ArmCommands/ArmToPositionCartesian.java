// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class ArmToPositionCartesian extends CommandBase {
  /** Creates a new ElbowToPosition. */
  private ArmSubsystem armSubsystem;
  private double xPos;
  private double yPos;
  
  public ArmToPositionCartesian(ArmSubsystem a, double x, double y) {
    armSubsystem = a;
    xPos = x;
    yPos = y;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("Is starting", true);
    System.out.println("Running");
    setAngle();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("End Command", true);
     armSubsystem.setElbowGoal(ArmConstants.elbowHome);
     armSubsystem.setShoulderGoal(ArmConstants.shoulderHome);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /*double proximalError = Math.abs(armSubsystem.getShoulderPosition() - armSubsystem.getShoulderGoal());
    double distalError = Math.abs(armSubsystem.getElbowPosition() - armSubsystem.getElbowGoal());

    SmartDashboard.putNumber("proximal error", proximalError);
    SmartDashboard.putNumber("distal error", distalError);

    if(Math.abs(armSubsystem.getShoulderPosition() - armSubsystem.getShoulderGoal()) < 0.6 
    && Math.abs(armSubsystem.getElbowPosition() - armSubsystem.getElbowGoal()) < 0.6){
      counter++;
    }
    else{
      counter = 0;
    }
    return counter >= counterThreshold;*/
    return false;
  }

  public double[] calculateCircleIntersection(double x1, double y1, double r1, double x2, double y2, double r2){
    double[] solutions = new double[2];
    double centerdx = x1 - x2;
    double centerdy = y1 - y2;
    double R = Math.sqrt(centerdx * centerdx + centerdy * centerdy);
    if (!(Math.abs(r1 - r2) <= R && R <= r1 + r2)) { // no intersection
      solutions[0] = 0;
      solutions[1] = 0;
      return solutions;
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

  public void setAngle(){
    double[] intersection = calculateCircleIntersection(0, 0, ArmConstants.lengthOfShoulder, xPos, yPos, ArmConstants.lengthOfElbow);
    double shoulderAngle = ArmConstants.shoulderHome; //setting these as default
    double elbowAngle = ArmConstants.elbowHome; //setting these as default
    if(xPos < 0){
      shoulderAngle = Math.PI - Math.asin(intersection[1] / ArmConstants.lengthOfShoulder);
      elbowAngle = Math.PI - Math.asin((yPos-intersection[1]) / ArmConstants.lengthOfElbow);

      //optimized movement for the distal
      elbowAngle-=(2*Math.PI);
    }
    else{
      shoulderAngle = Math.asin(intersection[1] / ArmConstants.lengthOfShoulder);
      elbowAngle = Math.asin((yPos-intersection[1]) / ArmConstants.lengthOfElbow);
    }
    System.out.println(shoulderAngle);
    System.out.println(elbowAngle);
    armSubsystem.setShoulderGoal(shoulderAngle);
    armSubsystem.setElbowGoal(elbowAngle); 
    
  }


}
