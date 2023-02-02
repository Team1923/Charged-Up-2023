package frc.robot.util;

import frc.robot.util.StateVariables.ArmPositions;
import frc.robot.util.StateVariables.CurrentRobotDirection;
import frc.robot.util.StateVariables.GamePieceMode;
import frc.robot.util.StateVariables.HorizontalLocations;
import frc.robot.util.StateVariables.IntakePositions;
import frc.robot.util.StateVariables.ScoringLocations;
import frc.robot.util.StateVariables.VerticalLocations;

public class StateHandler {
    private static StateHandler stateHandler;
    private ScoringLocations scoringLocations = new ScoringLocations(VerticalLocations.LOW, HorizontalLocations.LEFT);

    private ArmPositions desiredArmPosition = ArmPositions.STOW;
    private ArmPositions currentArmPosition = ArmPositions.STOW;

    private IntakePositions desiredIntakePosition = IntakePositions.STOW;

    private boolean hasGamePiece = true;
    private boolean intakeInPosition = false, armInPosition = false;
    private boolean resetManipulator = false;

    private GamePieceMode mode = GamePieceMode.CONE;
    
    private CurrentRobotDirection currentRobotDirection = CurrentRobotDirection.LEFT;

    public static synchronized StateHandler getInstance() {
        if (stateHandler == null) {
            stateHandler = new StateHandler();
        }
        return stateHandler;
    }

    public ScoringLocations getCurrentScoringLocation() {
        return scoringLocations;
    }

    public void setScoringLocation(VerticalLocations v, HorizontalLocations h) {
        scoringLocations.setVerticalLocation(v);
        scoringLocations.setHorizontalLocation(h);
    }

    public void updateArmDesiredState(ArmPositions a) {
        if(a != desiredArmPosition){
            armInPosition = false;
        }
        desiredArmPosition = a;
    }

    public ArmPositions getArmDesiredPosition() {
        return desiredArmPosition;
    }

    public void updateIntakeInPosition(boolean intakeInPosition) {
        this.intakeInPosition = intakeInPosition;
    }

    public void updateArmInPosition(boolean armInPosition) {
        this.armInPosition = armInPosition;
    }

    public void updateHasGamePiece(boolean hasGamePiece) {
        this.hasGamePiece = hasGamePiece;
    }

    public void updateManipulator(boolean resetManipulator) {
        this.resetManipulator = resetManipulator;
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
        return intakeInPosition && armInPosition;
    }

    public boolean getResetManipulator() {
        return resetManipulator;
    }

    public CurrentRobotDirection getRobotDirection() {
        return currentRobotDirection;
    }

    public void updateRobotDirection(CurrentRobotDirection c) {
        this.currentRobotDirection = c;
    }

    public ArmPositions getCurrentArmPosition() {
        return currentArmPosition;
    }

    public void setCurrentArmPosition(ArmPositions a) {
        this.currentArmPosition = a;
    }

    public IntakePositions getDesiredIntakePosition() {
        return desiredIntakePosition;
    }

    public void setDesiredIntakePosition(IntakePositions i) {
        if(i!=desiredIntakePosition){
            intakeInPosition = false;
        }
        this.desiredIntakePosition = i;
    }

    public GamePieceMode getGamePieceMode(){
        return mode;
    }

    public void setGamePieceMode(GamePieceMode g){
        mode = g;
    }

}
