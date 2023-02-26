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
        if(limelightInterface.hasScoringTarget()){
            ydeviation.add(limelightInterface.getBotPose(limelight)[1]);
        }
    }
    public void yawSample(SpecificLimelight limelight){
        if(limelightInterface.hasScoringTarget()){
            yawdeviation.add(limelightInterface.getBotPose(limelight)[5]);
        }
    }
    public void xMap(double distance){
        xstd.put(distance,calculateStandardDeviation(xdeviation));
    }
    public void yMap(double distance){
        ystd.put(distance,calculateStandardDeviation(ydeviation));
      }
    public void yawMap(double distance){
        yawstd.put(distance,calculateStandardDeviation(yawdeviation));
      }

    public void wipeSample(ArrayList<Double> sample){
      sample.clear();
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

     public void printMap(HashMap<Double, Double> map, double Distance){
      System.out.println("Distance Away: " + Distance + ", " + " Standard Deviation: " + map.get(Distance));
     }

     public ArrayList<Double> getXSample(){
      return xdeviation;
     }
     public ArrayList<Double> getYSample(){
      return ydeviation;
     }
     public ArrayList<Double> getYawSample(){
      return yawdeviation;
     }

     public HashMap<Double,Double> getXMap(){
      return xstd;
     }
     public HashMap<Double,Double> getYMap(){
      return ystd;
     }
     public HashMap<Double,Double> getYawMap(){
      return yawstd;
     }

    
     
    
}
