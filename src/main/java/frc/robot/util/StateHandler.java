package frc.robot.util;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.interfaces.LimelightInterface.SpecificLimelight;
import frc.robot.util.StateVariables.CurrentRobotDirection;
import frc.robot.util.StateVariables.GamePieceMode;
import frc.robot.util.StateVariables.IntakePositions;
import frc.robot.util.StateVariables.IntakeWheelSpeeds;
import frc.robot.util.StateVariables.VerticalLocations;

public class StateHandler {
    private static StateHandler stateHandler;
    private VerticalLocations verticalLocations = VerticalLocations.LOW;
    
    private IntakePositions desiredIntakePosition = IntakePositions.SHOOT_TALL;

    private CurrentRobotDirection currentRobotDirection = CurrentRobotDirection.RIGHT;

    private IntakeWheelSpeeds desiredIntakeWheelSpeed = IntakeWheelSpeeds.GRIP;

    private boolean hasGamePiece = false;

    private boolean isIntakeGood = false;


    private GamePieceMode currentGamePiece = GamePieceMode.CUBE;

    private boolean autoShootWheels = false;

    private boolean autoIntakeWheels = false;

    private boolean wantToBeHappy = false;

    private boolean useGyroVelocityMeasurement = false;

    private boolean stickOut = false;


    public static synchronized StateHandler getInstance() {
        if (stateHandler == null) {
            stateHandler = new StateHandler();
        }
        return stateHandler;
    }

   

    public VerticalLocations getCurrentVerticalLocation() {
        return verticalLocations;
    }

    public void setVerticalLocation(VerticalLocations v) {
        verticalLocations = v;
    }

    public boolean hasGamePiece(){
        return hasGamePiece;
    }

    public void setHasGamePiece(boolean setGamePiece){
        this.hasGamePiece = setGamePiece;
    }

    public GamePieceMode getGamePieceMode(){
        return currentGamePiece;
    }

    public void setGamePieceMode(GamePieceMode gamepiece){
        currentGamePiece = gamepiece;
    }

    public IntakePositions getDesiredIntakePosition() {
        return desiredIntakePosition;
    }

    public void setDesiredIntakePosition(IntakePositions i) {
        desiredIntakePosition = i;
    }

    public boolean getIntakeGood(){
        return isIntakeGood;
    }

    public void setIntakeGood(boolean setIntake){
        this.isIntakeGood = setIntake;
    }

    public SpecificLimelight getSpecificLimelight() {
        if (currentRobotDirection == CurrentRobotDirection.LEFT) {
            return SpecificLimelight.LEFT_LIMELIGHT;
        } else {
            return SpecificLimelight.RIGHT_LIMELIGHT;
        }
    }


    public boolean getAutoShootWheels() {
        return autoShootWheels;
    }

    public void setAutoShootWheels(boolean s) {
        autoShootWheels = s;
    }

    public boolean getAutoIntakeWheels() {
        return autoIntakeWheels;
    }

    public void setAutoIntakeWheels(boolean i) {
        autoIntakeWheels = i;
    }

    public IntakeWheelSpeeds getDesiredIntakeWheelSpeed() {
        return desiredIntakeWheelSpeed;
    } 

    public void setDesiredIntakeWheelSpeed(IntakeWheelSpeeds iWheelSpeeds) {
        desiredIntakeWheelSpeed = iWheelSpeeds;
    }

    public void setWantToBeHappy(boolean s) {
        wantToBeHappy = s;
    }

    public boolean getWantToBeHappy() {
        return wantToBeHappy;
    }

    public void setUseGyroVelocityMeasurement(boolean u) {
        useGyroVelocityMeasurement = u;
    }

    public boolean getUseGyroVelocityMeasurement() {
        return useGyroVelocityMeasurement;
    }

    public void resetAutoStates() {
        useGyroVelocityMeasurement = false;
        desiredIntakePosition = IntakePositions.SHOOT_TALL;
        desiredIntakeWheelSpeed = IntakeWheelSpeeds.GRIP;
    }

    public Value getStickOutSolenoid(){
        return this.stickOut ? Value.kReverse : Value.kForward;
    }

    public void setStickOut(boolean s){
        this.stickOut = s;
    }


}