package frc.robot.util;

import frc.robot.interfaces.LimelightInterface.SpecificLimelight;
import frc.robot.util.StateVariables.ArmPositions;
import frc.robot.util.StateVariables.CurrentRobotDirection;
import frc.robot.util.StateVariables.GamePieceMode;
import frc.robot.util.StateVariables.HorizontalLocations;
import frc.robot.util.StateVariables.IntakePositions;
import frc.robot.util.StateVariables.VerticalLocations;

public class StateHandler {
    private static StateHandler stateHandler;
    private VerticalLocations verticalLocations = VerticalLocations.LOW;
    private HorizontalLocations horizontalLocations = HorizontalLocations.LEFT;

    private ArmPositions desiredArmPosition = ArmPositions.STOW;
    private ArmPositions currentArmPosition = ArmPositions.STOW;

    private IntakePositions desiredIntakePosition = IntakePositions.STOW;
    private IntakePositions currentIntakePosition = IntakePositions.STOW;

    private boolean intakeInPosition = false,
            hasGamePiece = false,
            armInPosition = false,
            resetManipulator = false,
            gripperEngaged = false,
            wantToScore = false,
            holdInCobra = false,
            manualLift = false,
            armGood = false,
            intakeGood = false,
            autoRunIntake = false,
            wantToEngage = false,
            intakeInFeed = false;

    private GamePieceMode mode = GamePieceMode.CUBE;

    private CurrentRobotDirection currentRobotDirection = CurrentRobotDirection.RIGHT;
    private boolean isArmMoving = false;

    private double timeSinceLastGripChange = System.currentTimeMillis();

    private double timeSinceReadyToScore = 0;

    //private boolean resetManipulator = false;

    public static synchronized StateHandler getInstance() {
        if (stateHandler == null) {
            stateHandler = new StateHandler();
        }
        return stateHandler;
    }

    public boolean getIsArmMoving(){
        return isArmMoving;
    }

    public void setIsArmMoving(boolean isArmMoving){
        this.isArmMoving = isArmMoving;
    }

    public double getTimeSinceReadyToScore() {
        return timeSinceReadyToScore;
    }

    public void setTimeSinceReadyToScore(double time) {
        timeSinceReadyToScore = time;
    }

    public VerticalLocations getCurrentVerticalLocation() {
        return verticalLocations;
    }

    public void setVerticalLocation(VerticalLocations v) {
        verticalLocations = v;
    }

    public HorizontalLocations getCurrentHorizontalLocation() {
        return horizontalLocations;
    }

    public void setHorizontalLocation(HorizontalLocations h) {
        horizontalLocations = h;
    }

    public void setArmDesiredState(ArmPositions a) {
        if (a != desiredArmPosition) {
            armInPosition = false;
        }
        desiredArmPosition = a;
    }

    public ArmPositions getArmDesiredPosition() {
        return desiredArmPosition;
    }

    public void setIntakeInPosition(boolean intakeInPosition) {
        this.intakeInPosition = intakeInPosition;
    }

    public void setArmInPosition(boolean armInPosition) {
        this.armInPosition = armInPosition;
    }

    public void setHasGamePiece(boolean hasGamePiece) {
        this.hasGamePiece = hasGamePiece;
    }

    public boolean getIntakeInPosition() {
        return intakeInPosition;
    }

    public boolean getArmInPosition() {
        return armInPosition;
    }

    public boolean getHasGamePiece() {
        return hasGamePiece;
    }

    public boolean readyToClose() {
        return (currentIntakePosition == IntakePositions.FINAL_HANDOFF) &&
                (currentArmPosition == ArmPositions.STOW) && desiredIntakePosition == IntakePositions.FINAL_HANDOFF;
    }

    public void setResetManipulator(boolean resetManipulator) {
        this.resetManipulator = resetManipulator;
    }

    public boolean getResetManipulator() {
        return resetManipulator;
    }

    public CurrentRobotDirection getRobotDirection() {
        return currentRobotDirection;
    }

    public void setRobotDirection(CurrentRobotDirection c) {
        currentRobotDirection = c;
    }

    public ArmPositions getCurrentArmPosition() {
        return currentArmPosition;
    }

    public void setCurrentArmPosition(ArmPositions a) {
        currentArmPosition = a;
    }

    public IntakePositions getCurrentIntakePosition() {
        return currentIntakePosition;
    }

    public void setCurrentIntakePosition(IntakePositions i) {
        currentIntakePosition = i;
    }

    public IntakePositions getDesiredIntakePosition() {
        return desiredIntakePosition;
    }

    public void setDesiredIntakePosition(IntakePositions i) {
        if (i != desiredIntakePosition) {
            intakeInPosition = false;
        }
        desiredIntakePosition = i;
    }

    public GamePieceMode getGamePieceMode() {
        return mode;
    }

    public void setGamePieceMode(GamePieceMode g) {
        mode = g;
    }

    public void setGripperEngaged(boolean g) {
        gripperEngaged = g;
    }

    public boolean getGripperEngaged() {
        return gripperEngaged;
    }

    public SpecificLimelight getSpecificLimelight() {
        if (currentRobotDirection == CurrentRobotDirection.LEFT) {
            return SpecificLimelight.LEFT_LIMELIGHT;
        } else {
            return SpecificLimelight.RIGHT_LIMELIGHT;
        }
    }

    public void setTimeSinceLastGripChange() {
        timeSinceLastGripChange = System.currentTimeMillis();
    }

    // Time in seconds since the gripper was changed
    public double getTimeSinceLastGripChange() {
        return (System.currentTimeMillis() - timeSinceLastGripChange) / 1000;
    }

    public void setHoldInCobra(boolean set) {
        holdInCobra = set;
    }

    public boolean getHoldInCobra() {
        return holdInCobra;
    }

    public void setWantToScore(boolean set) {
        wantToScore = set;
    }

    public boolean getWantToScore() {
        return wantToScore;
    }

    public boolean getManualLift() {
        return manualLift;
    }

    public void setManualLift(boolean m) {
        manualLift = m;
    }
    
    public void setAutoRunIntake(boolean runIntake) {
        autoRunIntake = runIntake;
    }

    public boolean getAutoRunIntake(){
        return autoRunIntake;
    }


    public void setWantToEngage(boolean e) {
        wantToEngage = e;
    }

    public boolean getWantToEngage() {
        return wantToEngage;
    }

    public boolean getIntakeInFeed() {
        return intakeInFeed;
    }

    public void setIntakeInFeed(boolean f) {
        intakeInFeed = f;
    }
    
    // When the robot is disbaled, it resets the states of the Arm and Intake, preventing them from continuing their command after the robot is disabled
    public void resetStates(){
        desiredArmPosition = ArmPositions.STOW;
        currentArmPosition = ArmPositions.STOW;
    
        desiredIntakePosition = IntakePositions.STOW;
        currentIntakePosition = IntakePositions.STOW;

        intakeInPosition = false;
        hasGamePiece = false;
        resetManipulator = false;
        gripperEngaged = false;
        wantToScore = false;
        holdInCobra = false;
        manualLift = false;
        wantToEngage = false;
        
        currentRobotDirection = CurrentRobotDirection.RIGHT;

        isArmMoving = false;
        wantToEngage = false;
        intakeInFeed = false;
    
        timeSinceLastGripChange = System.currentTimeMillis();
    
        timeSinceReadyToScore = 0;
        
    }

    public void resetAutoState(){
        desiredArmPosition = ArmPositions.STOW;
        currentArmPosition = ArmPositions.STOW;
        // CHANGED FROM FINAL HANDOFF. IF BAD THEN UNDO ADDITIONS TO MANIPULATOR DEFAULT COMMANDS.
        desiredIntakePosition = IntakePositions.STOW;
        currentIntakePosition = IntakePositions.STOW;
        hasGamePiece = true;
        timeSinceLastGripChange = System.currentTimeMillis();
        timeSinceReadyToScore = 0;
        gripperEngaged = true;
        resetManipulator = false;
        wantToEngage = false;
        wantToEngage = false;
        intakeInFeed = false;


        isArmMoving = true;
        wantToScore = true;
        holdInCobra = false;
    }

    public void setArmGood(boolean isArmGood){
        armGood = isArmGood;
    }

    public boolean getIsArmGood(){
        return armGood;
    }

    public void setIntakeGood(boolean isIntakeGood){
        intakeGood = isIntakeGood;
    }

    public boolean getIsIntakeGood(){
        return intakeGood;
    }

    // Returns the current arm position from scoring location depending on where the current Vertical Location Is
    public ArmPositions getArmPositionFromScoringLocation() {
        VerticalLocations currentVerticalLocation = getCurrentVerticalLocation();
        ArmPositions armPositionFromScoringLocation = ArmPositions.STOW;
        switch (currentVerticalLocation) {
            case LOW:
                armPositionFromScoringLocation = ArmPositions.LOW;
                break;
            case MID:
                if (getGamePieceMode() == GamePieceMode.CUBE) {
                    armPositionFromScoringLocation = ArmPositions.CUBE_MID;
                } else {
                    armPositionFromScoringLocation = ArmPositions.CONE_MID;
                }
                break;
            case HIGH:
                if (getGamePieceMode() == GamePieceMode.CUBE) {
                    armPositionFromScoringLocation = ArmPositions.CUBE_HIGH;
                } else {
                    armPositionFromScoringLocation = ArmPositions.CONE_HIGH;
                }
                break;
            case RESET:
                armPositionFromScoringLocation = getCurrentArmPosition();
                break;
            default:
                break;
        }
        return armPositionFromScoringLocation;
    }
}