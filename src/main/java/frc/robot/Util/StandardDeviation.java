package frc.robot.Util;

import java.util.ArrayList;
import java.util.HashMap;

import frc.robot.interfaces.BetterLimelightInterface;

public class StandardDeviation {

    BetterLimelightInterface limelightInterface = BetterLimelightInterface.getInstance();

    ArrayList<Double> xstd = new ArrayList<>();
    ArrayList<Double> ystd = new ArrayList<>();
    ArrayList<Double> thetastd = new ArrayList<>();

    HashMap<Integer, Double> xdistance = new HashMap<>();
    HashMap<Integer, Double> ydistance = new HashMap<>();
    HashMap<Integer, Double> thetadistance = new HashMap<>();
    
    
}
