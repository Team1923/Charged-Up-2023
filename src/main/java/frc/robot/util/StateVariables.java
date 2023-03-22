package frc.robot.util;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class StateVariables {

    public static enum IntakeWheelSpeeds{
        INTAKE(new IntakeSpeed(0.5, 0.25)),
        SHOOT_FAR(new IntakeSpeed(-0.5, -1)),
        SHOOT_HIGH(new IntakeSpeed(-1, 0)),
        SHOOT_MID(new IntakeSpeed(-0.30, 0)),
        SHOOT_LOW(new IntakeSpeed(-0.15, 0)),
        GRIP(new IntakeSpeed(0.075, 0));

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
        INTAKE(new ArmAngles(Math.toRadians(-5)), new ArmAngles(Math.toRadians(-5)), true, true),
        SHOOT_TALL(new ArmAngles(2.132 + 0.17), new ArmAngles(2), true, false),
        SHOOT_SMALL(new ArmAngles(2.245 + 0.17), new ArmAngles(2.132 + 0.17), false, true);


        private ArmAngles mainArmAngle;
        private ArmAngles temporaryArmAngle;
        private boolean hardstopUp;
        private boolean horizontalEngaged;

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

        private IntakePositions(ArmAngles mainAngle, boolean hardstopTall, boolean hEngaged) {
            this.mainArmAngle = mainAngle;
            this.hardstopUp = hardstopTall;
            this.horizontalEngaged = hEngaged;
        }

        private IntakePositions(ArmAngles mainAngle, ArmAngles tempAngle, boolean hardstopTall, boolean hEngaged) {
            this.mainArmAngle = mainAngle;
            this.temporaryArmAngle = tempAngle;
            this.hardstopUp = hardstopTall;
            this.horizontalEngaged = hEngaged;
        }

        public ArmAngles getMainAngle() {
            return this.mainArmAngle;
        }

        public ArmAngles getTempAngle() {
            return this.temporaryArmAngle;
        }
        
        public Value getHardstopSolenoid() {
            return this.hardstopUp ? Value.kForward : Value.kReverse;
        }

        public Value getHorizontalSolenoid() {
            return this.horizontalEngaged ? Value.kReverse : Value.kForward;
        }

    }

    public static class IntakeSpeed{
        private double spd;
        private double horizontalRollerSpd;

        public IntakeSpeed(double s, double horizontalRollerSpd) {
            spd = s;
            this.horizontalRollerSpd = horizontalRollerSpd;
        }

        public double getWheelSpeed() {
            return spd;
        }

        public double getHorizontalRollerSpd() {
            return horizontalRollerSpd;
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
