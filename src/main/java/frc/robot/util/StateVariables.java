package frc.robot.util;

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

    public static enum CurrentRobotDirection {
        LEFT,
        RIGHT
    }

    public static enum ArmPositions {
        STOW(new ArmAngles(Math.PI / 2, -Math.PI / 2)),
        COBRA(new ArmAngles(Math.toRadians(110), 0)),
        CONE_LOW(new ArmAngles(1.815, 0.718)),
        CONE_MID(new ArmAngles(1.346, 0.466)),
        CONE_HIGH(new ArmAngles(0.986, 0.141)),
        CUBE_LOW(new ArmAngles(1.815, 0.718)),
        CUBE_MID(new ArmAngles(1.238, 0.669)),
        CUBE_HIGH(new ArmAngles(0.986, 0.141));

        private ArmAngles armAngles;
        private ArmAngles reflectedAngles;

        private ArmPositions nextInSequence;
        private double triggerThresholdRadians;

        /**
         * Enable the state to automatically switch to another state when a
         * certain angle has been reached.
         *
         * @param angles         Instance of ArmAngles containing arm angle data for
         *                       this waypoint
         * @param next           The next arm state in the sequence
         * @param triggerRadians The threshold (in radians) for the joint before moving
         *                       to next waypoint
         */
        private ArmPositions(ArmAngles angles, ArmPositions next, double triggerRadians) {
            this.armAngles = angles;
            this.reflectedAngles = new ArmAngles(Math.PI - armAngles.getProximalAngle(),
                    Math.PI - armAngles.getDistalAngle());
            this.nextInSequence = next;
            this.triggerThresholdRadians = triggerRadians;
        }

        private ArmPositions(ArmAngles angles) {
            this.armAngles = angles;
            this.reflectedAngles = new ArmAngles(Math.PI - armAngles.getProximalAngle(),
                    Math.PI - armAngles.getDistalAngle());
        }

        public ArmAngles getArmAngles() {
            return armAngles;
        }

        public ArmAngles getReflectedArmAngles() {
            return reflectedAngles;
        }

        public double getThresholdRadians() {
            return triggerThresholdRadians;
        }

        public ArmPositions getNextInSequence() {
            return nextInSequence;
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
        private double proximalAngle;
        private double distalAngle;

        public ArmAngles(double s, double e) {
            proximalAngle = s;
            distalAngle = e;
        }

        public void setProximalAngle(double s) {
            proximalAngle = s;
        }

        public void setDistalAngle(double e) {
            distalAngle = e;
        }

        public double getProximalAngle() {
            return proximalAngle;
        }

        public double getDistalAngle() {
            return distalAngle;
        }
    }
}
