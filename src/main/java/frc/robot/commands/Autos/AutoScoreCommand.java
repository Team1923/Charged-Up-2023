// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.util.StateHandler;
import frc.robot.util.StateVariables.ArmPositions;
import frc.robot.util.StateVariables.GamePieceMode;
import frc.robot.util.StateVariables.HorizontalLocations;
import frc.robot.util.StateVariables.VerticalLocations;

public class AutoScoreCommand extends CommandBase {
  /** Creates a new AutoScoreCommand. */
  private StateHandler stateHandler = StateHandler.getInstance();
  private HorizontalLocations desiredHorizontalLocation;
  private VerticalLocations desiredVerticalLocation;
  private GamePieceMode desiredGamePieceMode;
  public AutoScoreCommand(HorizontalLocations horizontalLocation, VerticalLocations verticalLocation, 
    GamePieceMode gamePieceMode) {
    desiredHorizontalLocation = horizontalLocation;
    desiredVerticalLocation = verticalLocation;
    desiredGamePieceMode = gamePieceMode;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stateHandler.setVerticalLocation(desiredVerticalLocation);
    stateHandler.setHorizontalLocation(desiredHorizontalLocation);
    stateHandler.setGamePieceMode(desiredGamePieceMode);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    stateHandler.setResetManipulator(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return stateHandler.getArmPositionFromScoringLocation() == stateHandler.getCurrentArmPosition();
  }
}
