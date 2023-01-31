package frc.robot.util;

import java.util.HashMap;
import java.util.Map;

import frc.robot.Constants.ArmConstants;

public class StateVariables {
    public static enum VerticalLocations {
        LOW,
        MID,
        HIGH
    }

    public static enum HorizontalLocations {
        LEFT,
        CENTER,
        RIGHT
    }

    public static enum GamePieceMode {
        CUBE,
        CONE
    }

    public static enum ArmPositions{
        STOW(new ArmAngles(Math.PI/2,-Math.PI/2)),
        COBRA(new ArmAngles(Math.toRadians(110),0)), 
        CONE_LOW(new ArmAngles(1.815, 0.718)),
        CONE_MID(new ArmAngles(1.346, 0.466)),
        CONE_HIGH(new ArmAngles(0.986, 0.141)),
        CUBE_LOW(new ArmAngles(1.815, 0.718)),
        CUBE_MID(new ArmAngles(1.238, 0.669)),
        CUBE_HIGH(new ArmAngles(0.986, 0.141));

        private static final Map<ArmAngles, ArmPositions> BY_ANGLES = new HashMap<>();
        private ArmAngles armAngles;
        private ArmPositions(ArmAngles a){
            this.armAngles = a;
        }
        static{
            for(ArmPositions a : values()){
                BY_ANGLES.put(a.armAngles, a);
            }
        }

        public ArmAngles getArmAngles(){
            return armAngles;
        }
    }

    public static class ScoringLocations {
        private VerticalLocations verticalLocation;
        private HorizontalLocations horizontalLocation;

        public ScoringLocations(VerticalLocations v, HorizontalLocations h) {
            verticalLocation = v;
            horizontalLocation = h;
        }

        public void setVerticalLocation(VerticalLocations v) {
            verticalLocation = v;
        }

        public void setHorizontalLocation(HorizontalLocations h) {
            horizontalLocation = h;
        }

        public VerticalLocations getVerticalLocation() {
            return verticalLocation;
        }

        public HorizontalLocations getHoriontalLocation() {
            return horizontalLocation;
        }
    }

    public static class ArmAngles {
        private double shoulderAngle;
        private double elbowAngle;

        public ArmAngles(double s, double e) {
            shoulderAngle = s;
            elbowAngle = e;
        }

        public void setShoulderAngle(double s) {
            shoulderAngle = s;
        }

        public void setElbowAngle(double e) {
            elbowAngle = e;
        }

        public double getShoulderAngle() {
            return shoulderAngle;
        }

        public double getElbowAngle() {
            return elbowAngle;
        }
    }
}
