package frc.robot.util;

public class StateVariables {
    
    public static enum ArmPositions {
        STOW(new ArmAngles(Math.PI / 2, -Math.PI / 2)),
        COBRA_FORWARD(new ArmAngles(Math.toRadians(110), 0)),
        COBRA_REVERSE(new ArmAngles(Math.toRadians(110), 0)),
        LOW(new ArmAngles(0.773, -1.843)),
        CONE_MID(new ArmAngles(1.760, -0.289)),
        CONE_HIGH(new ArmAngles(2.202, 0.330)),
        CUBE_MID(new ArmAngles(1.242, -0.688)),
        CUBE_HIGH(new ArmAngles(Math.toRadians(60.864), Math.toRadians(-30.51)));

        private ArmAngles armAngles;
        private ArmAngles reflectedAngles;

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
            this.reflectedAngles = new ArmAngles(Math.PI - armAngles.getProximalAngle(),
                    (-Math.PI) - armAngles.getDistalAngle());
            this.nextInSequence = next;
            this.triggerThresholdRadians = triggerRadians;
        }

        private ArmPositions(ArmAngles angles) {
            this.armAngles = angles;
            this.reflectedAngles = new ArmAngles(Math.PI - armAngles.getProximalAngle(),
                    (-Math.PI) - armAngles.getDistalAngle());
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

    public static enum IntakePositions {
        INTAKE(new ArmAngles(0, 0)),
        HANDOFF_1(new ArmAngles(1.249, Math.toRadians(40))),
        HANDOFF_2(new ArmAngles(2.541, 0.225)),
        FINAL_HANDOFF(new ArmAngles(2.871, 1.347 + Math.toRadians(15))),
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
