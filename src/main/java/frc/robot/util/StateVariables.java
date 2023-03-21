package frc.robot.util;


public class StateVariables {

    public static enum IntakeWheelSpeeds{
        INTAKE(new IntakeSpeed(0.5, 0.25, true)),
        SHOOT_HIGH(new IntakeSpeed(-1, 0, false)),
        SHOOT_MID(new IntakeSpeed(-0.35, 0, false)),
        SHOOT_LOW(new IntakeSpeed(-0.2, 0, false)),
        GRIP(new IntakeSpeed(0.1, 0, false));

        private IntakeSpeed iWheelSpeed;

        private IntakeWheelSpeeds(IntakeSpeed i) {
            this.iWheelSpeed = i;
        }

        public IntakeSpeed getIntakeWheelSpeed() {
            return iWheelSpeed;
        }
    }
    

    public static enum IntakePositions {
        // Similar to Arm Positions, diferent Intake Arm States take in Arm Angle Object
        INTAKE(new ArmAngles(0)),
        STOW(new ArmAngles(1.359));


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

    public static class IntakeSpeed{
        private double spd;
        private double horizontalRollerSpd;
        private boolean engageRollers;

        public IntakeSpeed(double s, double horizontalRollerSpd, boolean engageRollers) {
            spd = s;
            this.horizontalRollerSpd = horizontalRollerSpd;
            this.engageRollers = engageRollers;
        }

        public double getWheelSpeed() {
            return spd;
        }

        public double getHorizontalRollerSpd() {
            return horizontalRollerSpd;
        }

        public boolean getEngageRollers() {
            return engageRollers;
        }

    }

    public static class ArmAngles {
        private double angle;

        public ArmAngles(double a) {
            angle = a;
        }

        public void setAngle(double a) {
            angle = a;
        }

        public double getAngle() {
            return angle;
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

}
