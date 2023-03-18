package frc.robot.util;

import frc.robot.interfaces.LimelightInterface.SpecificLimelight;
import frc.robot.util.StateVariables.CurrentRobotDirection;
import frc.robot.util.StateVariables.GamePieceMode;
import frc.robot.util.StateVariables.IntakePositions;
import frc.robot.util.StateVariables.VerticalLocations;

public class StateHandler {
    private static StateHandler stateHandler;
    private VerticalLocations verticalLocations = VerticalLocations.LOW;
    
    private IntakePositions desiredIntakePosition = IntakePositions.STOW;
    private IntakePositions currentIntakePosition = IntakePositions.STOW;

    private CurrentRobotDirection currentRobotDirection = CurrentRobotDirection.RIGHT;

    private boolean hasGamePiece = false;

    private boolean isIntakeGood = false;

    private GamePieceMode currentGamePiece = GamePieceMode.CUBE;


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


    public void resetStates(){
        desiredIntakePosition = IntakePositions.STOW;
        currentIntakePosition = IntakePositions.STOW;
        currentRobotDirection = CurrentRobotDirection.RIGHT;
    }

    public void resetAutoState(){
        desiredIntakePosition = IntakePositions.STOW;
        currentIntakePosition = IntakePositions.STOW;
    }



}