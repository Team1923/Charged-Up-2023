package frc.robot.util;

import frc.robot.interfaces.ColorSensorInterface.GamePiece;
import frc.robot.util.StateVariables.ArmPositions;
import frc.robot.util.StateVariables.GamePieceMode;
import frc.robot.util.StateVariables.HorizontalLocations;
import frc.robot.util.StateVariables.ScoringLocations;
import frc.robot.util.StateVariables.VerticalLocations;

public class StateHandler {
    private static StateHandler stateHandler;
    private ScoringLocations scoringLocations = new ScoringLocations(VerticalLocations.LOW, HorizontalLocations.LEFT);
    private ArmPositions armPositions = ArmPositions.STOW;
    private boolean hasGamePiece = true;
    private boolean intakeInPosition = false, armInPosition = false;
    private GamePieceMode mode = GamePieceMode.CONE;
    private boolean resetManipulator = false;
    

    public static synchronized StateHandler getInstance(){
        if(stateHandler == null){
            stateHandler = new StateHandler();
        }
        return stateHandler;
    }

    public ScoringLocations getCurrentScoringLocation(){
        return scoringLocations;
    }

    public void setScoringLocation(VerticalLocations v, HorizontalLocations h){
        scoringLocations.setVerticalLocation(v);
        scoringLocations.setHorizontalLocation(h);
    }

    public void updateArmPosition(ArmPositions a){
        armPositions = a;
    }
    public ArmPositions getArmPositions(){
        return armPositions;
    }

    public void updateIntakePosition(boolean intakeInPosition) {
        this.intakeInPosition = intakeInPosition;

    }

    public void updateArmPosition(boolean armInPosition) {
        this.armInPosition = armInPosition;

    }

    public void updateHasGamePiece(boolean hasGamePiece) {
        this.hasGamePiece = hasGamePiece;

    }

    public void updateManipulator(boolean resetManipulator) {
        this.resetManipulator = resetManipulator;

    }

    public boolean getIntakePosition() {
        return intakeInPosition;

    }

    public boolean getArmPosition() {
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

}
