package frc.robot.interfaces;

import java.util.LinkedList;
import java.util.List;

import org.opencv.photo.Photo;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class PhotonVisionInterface {
    private PhotonCamera camera;
    private PhotonPipelineResult result;

    public PhotonVisionInterface(PhotonCamera camera){
        this.camera = camera;
    }

    public double getDistance(){
        if (result.hasTargets()) {
            // First calculate range
            double range =
                    PhotonUtils.calculateDistanceToTargetMeters(
                            Constants.CAMERA_HEIGHT_METERS,
                            Constants.TARGET_HEIGHT_METERS,
                            Constants.CAMERA_PITCH_RADIANS,
                            Units.degreesToRadians(result.getBestTarget().getPitch()));

        return range;
        }

        return 0;


        
    }
    

    public void updateResult(PhotonPipelineResult result){
        this.result = result;
    }

    public boolean hasTarget(){
        return this.result.hasTargets();
    }

    public List<PhotonTrackedTarget> getTargetList(){
        return result.getTargets();
    }

    public PhotonTrackedTarget getBestTarget(){
        return result.getBestTarget();
    }

    public double getYaw(PhotonTrackedTarget target){
        return target.getYaw();
    }

    public double getPitch(PhotonTrackedTarget target){
        return target.getPitch();
    }

    public double getArea(PhotonTrackedTarget target){
        return target.getArea();
    }

    public List<TargetCorner> getTargetCorner(PhotonTrackedTarget target){
        return target.getDetectedCorners();
    }

    public double getSkew(PhotonTrackedTarget target){
        return target.getSkew();
    }

    public double getTagNumber(PhotonTrackedTarget target){
        return target.getFiducialId();
    }

    public List<Integer> getTargetIDList(){
        List<PhotonTrackedTarget> target = getTargetList();
        List<Integer> ints = new LinkedList<Integer>();
        for(int i = 0; i < target.size(); i++){
            ints.add(target.get(i).getFiducialId());
        }

        return ints;
    }

    public Translation2d getTranslationToTarget(PhotonTrackedTarget target){
        return PhotonUtils.estimateCameraToTargetTranslation(getDistance(), Rotation2d.fromDegrees(-getYaw(target)));
    }

    public Pose2d getRobotPoseEstimation(PhotonTrackedTarget target){
        return PhotonUtils.estimateFieldToRobot(Constants.CAMERA_HEIGHT_METERS, Constants.TARGET_HEIGHT_METERS, Constants.CAMERA_PITCH_RADIANS, getPitch(target), new Rotation2d(getYaw(target)), new Rotation2d()/*gyroAngle*/, new Pose2d()/*fieldToTarget*/, new Transform2d() /*cameraToRobot*/);
    }





}
