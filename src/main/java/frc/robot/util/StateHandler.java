package frc.robot.util;

import frc.robot.util.StateVariables.ArmPositions;
import frc.robot.util.StateVariables.HorizontalLocations;
import frc.robot.util.StateVariables.ScoringLocations;
import frc.robot.util.StateVariables.VerticalLocations;

public class StateHandler {
    private static StateHandler stateHandler;
    private ScoringLocations scoringLocations = new ScoringLocations(VerticalLocations.LOW, HorizontalLocations.LEFT);
    private ArmPositions armPositions = ArmPositions.STOW;
    private boolean hasGamePiece = true;
    private boolean readyForHandoff = false;
    

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

    


}
