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
        STOW(new ArmAngles(1.616, -1.537), new ArmAngles(1.616, -1.537)),
        COBRA_FORWARD(new ArmAngles(Math.toRadians(110), -Math.PI/4), new ArmAngles(Math.toRadians(70), -3*Math.PI/4)),
        COBRA_REVERSE(new ArmAngles(Math.toRadians(90), -Math.PI/4), new ArmAngles(Math.toRadians(90), -3*Math.PI/4)),
        INVERTED_COBRA_REVERSE(new ArmAngles(Math.toRadians(90), -3*Math.PI/4), new ArmAngles(Math.toRadians(90), -Math.PI/4)),
        //DEFINE SCORING LOCATIONS: right is the first object, left is second
        LOW(new ArmAngles(1.473, -0.830), new ArmAngles(1.632, -2.107)),
        // CONE_MID(new ArmAngles(1.430, -0.327), new ArmAngles(1.722, -2.765)),
        // CONE_HIGH(new ArmAngles(1.005, 0.172), new ArmAngles(2.160 - Math.toRadians(5), -3.304 - Math.toRadians(2))),
        CONE_MID(new ArmAngles(1.437, -0.355), new ArmAngles(1.722, -2.765)),
        CONE_HIGH(new ArmAngles(1.043, 0.153), new ArmAngles(2.160 - Math.toRadians(5), -3.304 - Math.toRadians(2))),
        CUBE_MID(new ArmAngles(1.430, -0.327), new ArmAngles(1.722, -2.765)),
        CUBE_HIGH(new ArmAngles(1.005, 0.172), new ArmAngles(2.160 - Math.toRadians(5), -3.304 - Math.toRadians(2))),
        FEED(new ArmAngles(1.398 + Math.toRadians(1), -2.666 - Math.toRadians(7.5)), new ArmAngles(1.782 - Math.toRadians(2.75), -0.302 - Math.toRadians(5))); //THIS IS INVERTED FOR A REASON
        

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
        // Similar to Arm Positions, diferent Intake Arm States take in Arm Angle Object
        INTAKE(new ArmAngles(-Math.toRadians(15), Math.toRadians(-10))),
        EJECT(new ArmAngles(2.448, 0.811)),
        CLEAR(new ArmAngles(1.841,-0.324)),
        STOW(new ArmAngles(2.968, 1.359)),
        HANDOFF_1(new ArmAngles(1.249, Math.toRadians(40))),
        HANDOFF_2(new ArmAngles(2.541, 0.225)),
        FINAL_HANDOFF(new ArmAngles(2.979, 1.503)),
        /*TO DO: determine the reverse waypoints necessary to go to STOW */
        REVERSE_HANDOFF_1(new ArmAngles(1.249, Math.toRadians(40))),
        REVERSE_HANDOFF_2(new ArmAngles(0.246, 0.661));

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
    //Sets the Postions in Radians of the proximal and the distal arm for both the intake and the arm
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

    //Sets the Scoring Location based on its positon in the 3 by 3 grid, i.e Middle + Left
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
