package frc.robot.util;

public class StateVariables {
    
    public static enum ArmPositions {
        /**
         * New constructor for scoring locations.
         * All major scoring locations will take in 
         * 2 ArmAngle objects for sake of consistency
         * when scoring. 
         * 
         * NOTE: DUPLICATE VALUE IF ONLY ONE LOCATION
         */
        STOW(new ArmAngles(Math.PI / 2, -Math.PI / 2), new ArmAngles(Math.PI / 2, -Math.PI / 2)),
        COBRA_FORWARD(new ArmAngles(Math.toRadians(110), 0), new ArmAngles(Math.toRadians(110), 0)),
        COBRA_REVERSE(new ArmAngles(Math.toRadians(110), 0), new ArmAngles(Math.toRadians(110), 0)),
        //DEFINE SCORING LOCATIONS: right is the first object, left is second
        LOW(new ArmAngles(0, 0), new ArmAngles(0, 0)),
        CONE_MID(new ArmAngles(0, 0), new ArmAngles(0, 0)),
        CONE_HIGH(new ArmAngles(0, 0), new ArmAngles(0, 0)),
        CUBE_MID(new ArmAngles(0, 0), new ArmAngles(0, 0)),
        CUBE_HIGH(new ArmAngles(Math.toRadians(60.864), Math.toRadians(-30.51)), new ArmAngles(0,0));

        private ArmAngles armAngles;
        private ArmAngles leftArmAngles;

        private ArmPositions nextInSequence;
        private double triggerThresholdRadians;

        /**
         * Enable the state to automatically switch to another state once it
         * reaches its goal.
         *
         * @param angles         Instance of ArmAngles containing arm angle data for
         *                       this waypoint
         * @param next           The next arm state in the sequence
         * @param triggerRadians The threshold (in radians) for the joint before moving
         *                       to next waypoint
         */
        private ArmPositions(ArmAngles angles, ArmPositions next, double triggerRadians) {
            this.armAngles = angles;
            this.leftArmAngles = new ArmAngles(Math.PI - armAngles.getProximalAngle(),
                    (-Math.PI) - armAngles.getDistalAngle());
            this.nextInSequence = next;
            this.triggerThresholdRadians = triggerRadians;
        }

        private ArmPositions(ArmAngles angles) {
            this.armAngles = angles;
        }
        private ArmPositions(ArmAngles angles, ArmAngles leftArmAngles){
            this.armAngles = angles;
            this.leftArmAngles = leftArmAngles;
        }

        public ArmAngles getArmAngles() {
            return armAngles;
        }

        public ArmAngles getLeftArmAngles() {
            return leftArmAngles;
        }

        public double getThresholdRadians() {
            return triggerThresholdRadians;
        }

        public ArmPositions getNextInSequence() {
            return nextInSequence;
        }
        
    }

    public static enum IntakePositions {
        INTAKE(new ArmAngles(0, 0)),
        STOW(new ArmAngles(2.871, 1.347 + Math.toRadians(15))),
        HANDOFF_1(new ArmAngles(1.249, Math.toRadians(40))),
        HANDOFF_2(new ArmAngles(2.541, 0.225)),
        FINAL_HANDOFF(new ArmAngles(2.871, 1.347 + Math.toRadians(15))),
        /*TO DO: determine the reverse waypoints necessary to go to STOW */
        REVERSE_HANDOFF_1(new ArmAngles(1.249, Math.toRadians(40))),
        REVERSE_HANDOFF_2(new ArmAngles(2.541, 0.225));

        private ArmAngles armAngles;
        private IntakePositions nextInSequence;
        private double triggerThresholdRadians;

        /**
         * Enable the state to automatically switch to another state once it
         * reaches its goal.
         *
         * @param angles         Instance of ArmAngles containing arm angle data for
         *                       this waypoint
         * @param next           The next arm state in the sequence
         * @param triggerRadians The threshold (in radians) for the joint before moving
         *                       to next waypoint
         */
        private IntakePositions(ArmAngles angles, IntakePositions next, double triggerRadians) {
            this.armAngles = angles;
            this.nextInSequence = next;
            this.triggerThresholdRadians = triggerRadians;
        }

        private IntakePositions(ArmAngles angles) {
            this.armAngles = angles;

        }

        public ArmAngles getArmAngles() {
            return armAngles;
        }

        public double getThresholdRadians() {
            return triggerThresholdRadians;
        }

        public IntakePositions getNextInSequence() {
            return nextInSequence;
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


    public static enum VerticalLocations {
        LOW,
        MID,
        HIGH,
        RESET
    }

    public static enum HorizontalLocations {
        LEFT,
        CENTER,
        RIGHT,
        RESET
    }

    public static enum GamePieceMode {
        CUBE,
        CONE
    }

    public static enum CurrentRobotDirection {
        LEFT,
        RIGHT
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
}
