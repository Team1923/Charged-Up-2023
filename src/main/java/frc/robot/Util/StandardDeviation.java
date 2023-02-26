package frc.robot.Util;

import java.util.ArrayList;
import java.util.HashMap;


import frc.robot.interfaces.BetterLimelightInterface;
import frc.robot.interfaces.BetterLimelightInterface.SpecificLimelight;

public class StandardDeviation {

    BetterLimelightInterface limelightInterface = BetterLimelightInterface.getInstance();

    ArrayList<Double> xdeviation = new ArrayList<>();
    ArrayList<Double> ydeviation = new ArrayList<>();
    ArrayList<Double> yawdeviation = new ArrayList<>();

    HashMap<Double, Double> xstd = new HashMap<>();
    HashMap<Double, Double> ystd = new HashMap<>();
    HashMap<Double, Double> yawstd = new HashMap<>();

    public void xSample(SpecificLimelight limelight){
        if(limelightInterface.hasScoringTarget()){
            xdeviation.add(limelightInterface.getBotPose(limelight)[0]);
        }
    }
    public void ySample(SpecificLimelight limelight){
        while(limelightInterface.hasScoringTarget()){
            xdeviation.add(limelightInterface.getBotPose(limelight)[1]);
        }
    }
    public void yawSample(SpecificLimelight limelight){
        while(limelightInterface.hasScoringTarget()){
            xdeviation.add(limelightInterface.getBotPose(limelight)[5]);
        }
    }
    public void xMap(SpecificLimelight limelight){

      //calculate std deviation

      xstd.put(limelightInterface.getBotPose(limelight)[2],calculateStandardDeviation(xdeviation));

    }
    public void yMap(SpecificLimelight limelight){
        while(limelightInterface.hasScoringTarget()){
          ystd.put(limelightInterface.getBotPose(limelight)[2],calculateStandardDeviation(ydeviation));
        }
      }
      public void yawMap(SpecificLimelight limelight){
        while(limelightInterface.hasScoringTarget()){
          yawstd.put(limelightInterface.getBotPose(limelight)[2],calculateStandardDeviation(yawdeviation));
        }
      }

     public double calculateStandardDeviation(ArrayList<Double> Sample){
        double StandardDeviation =0 , sum = 0;
        for(double n: Sample){
            sum += n; 
        }
        for(double n : Sample){
                StandardDeviation += Math.pow(n - (sum/Sample.size()), 2);
            }

            return Math.sqrt(StandardDeviation/Sample.size());
     }
    
     
    
}
