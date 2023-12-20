package frc.robot.util;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class StateVariables {

    public static enum IntakeWheelSpeeds{
        INTAKE(new IntakeSpeed(0.5, 0.25)),
        SHOOT_HIGH(new IntakeSpeed(-0.8, 0)),
        SHOOT_MID(new IntakeSpeed(-0.30, 0)),
        SHOOT_LOW(new IntakeSpeed(-0.15, -0.15)),
        GRIP(new IntakeSpeed(0.075, 0)),
        EJECT(new IntakeSpeed(-0.5, -0.5)),
        PLOP_HIGH(new IntakeSpeed(-0.60, 0)),
        PLOP_MID(new IntakeSpeed(-0.295, 0)),
        CHARGE_STATION_PLOP(new IntakeSpeed(-1, 0)),
        HIGH_INTAKE_EJECT(new IntakeSpeed(-1,0)), 
        FIRST_AUTO_SHOT_BLUE(new IntakeSpeed(-0.85, -1, 0)),
        FIRST_AUTO_SHOT_RED(new IntakeSpeed(-1, -0.5, 0));


        private IntakeSpeed iWheelSpeed;

        private IntakeWheelSpeeds(IntakeSpeed i) {
            this.iWheelSpeed = i;
        }


        public IntakeSpeed getIntakeWheelSpeed() {
            return iWheelSpeed;
        }
    }
    
    public static enum IntakeWheelPositions{
        FREE(new IntakePos(-1)),
        POS_180(new IntakePos(180)),
        POS_0(new IntakePos(0));


        private IntakePos position;

        private IntakeWheelPositions(IntakePos pos){
            position = pos;
        }

        public double getPos(){
            return position.getIntakePos();
        }
    }

    public static enum IntakePositions {
        // Similar to Arm Positions, diferent Intake Arm States take in Arm Angle Object
        INTAKE(new ArmAngles(Math.toRadians(5)), new ArmAngles(Math.toRadians(5)), true, () -> StateHandler.getInstance().getDesiredIntakeWheelSpeed() == IntakeWheelSpeeds.INTAKE),
        INTAKE_RESET(new ArmAngles(Math.toRadians(-5)), new ArmAngles(Math.toRadians(-5)), true, () -> true),
        //INTAKE_BAR_UP(new ArmAngles(Math.toRadians(-5)), new ArmAngles(Math.toRadians(-5)), true, false),
        SHOOT_TALL(new ArmAngles(2.132 + 0.17 - Math.toRadians(4)), new ArmAngles(2), true, () -> false),
        SHOOT_SMALL(new ArmAngles(2.245 + 0.17), new ArmAngles(2.132 + 0.17), false, () -> false),
        GOOFY_SHOT(new ArmAngles(0.725), new ArmAngles(0.725), false, () -> false),
        PLOP_SHOT(new ArmAngles(2.132 - Math.toRadians(8)), new ArmAngles(2), true, () -> false),
        INTAKE_HIGHER(new ArmAngles(Math.toRadians(40)), new ArmAngles(Math.toRadians(40)), true, () -> false);

        private ArmAngles mainArmAngle;
        private ArmAngles temporaryArmAngle;
        private boolean hardstopUp;
        private BooleanSupplier horizontalEngaged;
    

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

        private IntakePositions(ArmAngles mainAngle, boolean hardstopTall, BooleanSupplier hEngaged) {
            this.mainArmAngle = mainAngle;
            this.hardstopUp = hardstopTall;
            this.horizontalEngaged = hEngaged;
            
        }

        private IntakePositions(ArmAngles mainAngle, ArmAngles tempAngle, boolean hardstopTall, BooleanSupplier hEngaged) {
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
            return this.horizontalEngaged.getAsBoolean() ? Value.kReverse : Value.kForward;
        }

       

    }

    public static class IntakePos{
        private double radians;

        public IntakePos(double r){
            radians=r;
        }

        public double getIntakePos(){
            return radians;
        }
    }

    public static class IntakeSpeed{
        private double spd;
        private double leftSpd;
        private double rightSpd;
        private double horizontalRollerSpd;

        public IntakeSpeed(double s, double horizontalRollerSpd) {
            leftSpd = s;
            rightSpd = s;
            this.horizontalRollerSpd = horizontalRollerSpd;
        }

        public IntakeSpeed(double leftSpd, double rightSpd, double horizontalRollerSpd) {
            this.leftSpd = leftSpd;
            this.rightSpd = rightSpd;
            this.horizontalRollerSpd = horizontalRollerSpd;
        }

        public double getHorizontalRollerSpd() {
            return horizontalRollerSpd;
        }

        public double getLeftSpeed() {
            return leftSpd;
        }

        public double getRightSpeed() {
            return rightSpd;
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
