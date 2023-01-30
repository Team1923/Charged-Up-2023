package frc.robot.interfaces;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LeftLimelightInterface {
    //declare for easy calls
    private static BetterLimelightInterface limelightInterface;

    //network table declarations
    private static NetworkTable leftLimelight = NetworkTableInstance.getDefault().getTable("limelight-left");

    public static synchronized BetterLimelightInterface getInstance(){
        if(limelightInterface == null){
            limelightInterface = new BetterLimelightInterface();
        }
        return limelightInterface;
    }

      public double getDoubleEntry(String entry){
          return leftLimelight.getEntry(entry).getDouble(0);
      }

      public double[] getArrayEntry(String entry){
          return leftLimelight.getEntry(entry).getDoubleArray(new double[6]);
      }

      /*
       * the only setting we really need to do on the limelight
       * is the pipeline (if we decide to vision track)
       */
      public void setPipeline(int pipeline){
          leftLimelight.getEntry("pipeline").setNumber(pipeline);
      }

      public boolean hasValidTargets(){
        return getDoubleEntry("tv") == 1.0;
      }

      public double getTargetArea(){
        return getDoubleEntry("ta");
      }

      /*
       * Specific AprilTag methods we will need
       */
      public double[] botPose(){
        return getArrayEntry("botpose");
      }

      public double getID(){
        return getDoubleEntry("tid");
      }

      /*
       * create a Pose3D object for trajectory generation
       */
      public Pose3d robotPose3d(){
        double[] result  = botPose();
        Translation3d tran3d = new Translation3d(result[0], result [1], result[2]);
        Rotation3d r3d = new Rotation3d(result[3], result[4], result[5]);
        Pose3d p3d = new Pose3d(tran3d, r3d);
    
        return p3d;
      }
    

}
