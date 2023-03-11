// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.StateHandler;
import frc.robot.util.StateVariables.ArmPositions;
import frc.robot.util.StateVariables.CurrentRobotDirection;
import frc.robot.util.StateVariables.GamePieceMode;
import frc.robot.util.StateVariables.HorizontalLocations;
import frc.robot.util.StateVariables.IntakePositions;
import frc.robot.util.StateVariables.VerticalLocations;

public class ShuffleboardSubsystem extends SubsystemBase {
  private StateHandler stateHandler = StateHandler.getInstance();

  public ShuffleboardTab driverDashboard = Shuffleboard.getTab("Driver Dashboard");

  private GenericEntry bottomLeft = driverDashboard.add("Left LOW", false)
      .withSize(1, 1)
      .withPosition(0, 0)
      .withProperties(Map.of("Color when false", "#000000", "Color when true", "#17FC03"))
      .getEntry();

  private GenericEntry bottomMiddle = driverDashboard.add("Mid LOW", false)
      .withSize(1, 1)
      .withPosition(1, 0)
      .withProperties(Map.of("Color when false", "#000000", "Color when true", "#17FC03"))
      .getEntry();

  private GenericEntry bottomRight = driverDashboard.add("Right LOW", false)
      .withSize(1, 1)
      .withPosition(2, 0)
      .withProperties(Map.of("Color when false", "#000000", "Color when true", "#17FC03"))
      .getEntry();

  private GenericEntry middleLeft = driverDashboard.add("Left MIDDLE", false)
      .withSize(1, 1)
      .withPosition(0, 1)
      .withProperties(Map.of("Color when false", "#000000", "Color when true", "#17FC03"))
      .getEntry();

  private GenericEntry middleMid = driverDashboard.add("Mid MIDDLE", false)
      .withSize(1, 1)
      .withPosition(1, 1)
      .withProperties(Map.of("Color when false", "#000000", "Color when true", "#17FC03"))
      .getEntry();

  private GenericEntry middleRight = driverDashboard.add("Right MIDDLE", false)
      .withSize(1, 1)
      .withPosition(2, 1)
      .withProperties(Map.of("Color when false", "#000000", "Color when true", "#17FC03"))
      .getEntry();

  private GenericEntry highLeft = driverDashboard.add("Left HIGH", false)
      .withSize(1, 1)
      .withPosition(0, 2)
      .withProperties(Map.of("Color when false", "#000000", "Color when true", "#17FC03"))
      .getEntry();

  private GenericEntry highMiddle = driverDashboard.add("Middle HIGH", false)
      .withSize(1, 1)
      .withPosition(1, 2)
      .withProperties(Map.of("Color when false", "#000000", "Color when true", "#17FC03"))
      .getEntry();

  private GenericEntry highRight = driverDashboard.add("Right HIGH", false)
      .withSize(1, 1)
      .withPosition(2, 2)
      .withProperties(Map.of("Color when false", "#000000", "Color when true", "#17FC03"))
      .getEntry();

  private GenericEntry desiredGamePiece = driverDashboard.add("Game Piece", false)
      .withSize(1, 1)
      .withPosition(3, 0)
      .withProperties(Map.of("Color when false", "#7842f5", "Color when true", "#f5ef42"))
      .getEntry();

  private GenericEntry desiredArmPosition = driverDashboard.add("DESIRED ARM", ArmPositions.STOW.toString())
      .withSize(1, 1)
      .withPosition(3, 1)
      .getEntry();

  private GenericEntry currentArmPosition = driverDashboard
      .add("CURRENT ARM", ArmPositions.STOW.toString())
      .withSize(1, 1)
      .withPosition(3, 2)
      .getEntry();

  private GenericEntry currentRobotDirection = driverDashboard
      .add("R DIRECTION", CurrentRobotDirection.LEFT.toString())
      .withSize(1, 1)
      .withPosition(4, 2)
      .getEntry();

  private GenericEntry desiredIntakePosition = driverDashboard
      .add("DESIRED INTAKE", IntakePositions.STOW.toString())
      .withSize(1, 1)
      .withPosition(4, 0)
      .getEntry();

  private GenericEntry currentIntakePosition = driverDashboard
      .add("CURRENT INTAKE", IntakePositions.STOW.toString())
      .withSize(1, 1)
      .withPosition(4, 1)
      .getEntry();

  public ShuffleboardSubsystem() {
  }

  @Override
  public void periodic() {
    VerticalLocations currentVerticalLocation = stateHandler.getCurrentVerticalLocation();
    HorizontalLocations currentHorizontalLocation = stateHandler.getCurrentHorizontalLocation();

    if (currentHorizontalLocation == HorizontalLocations.LEFT
        && currentVerticalLocation == VerticalLocations.LOW) {
      bottomLeft.setBoolean(true);
    } else {
      bottomLeft.setBoolean(false);
    }

    if (currentHorizontalLocation == HorizontalLocations.CENTER
        && currentVerticalLocation == VerticalLocations.LOW) {
      bottomMiddle.setBoolean(true);
    } else {
      bottomMiddle.setBoolean(false);
    }

    if (currentHorizontalLocation == HorizontalLocations.RIGHT
        && currentVerticalLocation == VerticalLocations.LOW) {
      bottomRight.setBoolean(true);
    } else {
      bottomRight.setBoolean(false);
    }

    if (currentHorizontalLocation == HorizontalLocations.LEFT
        && currentVerticalLocation == VerticalLocations.MID) {
      middleLeft.setBoolean(true);
    } else {
      middleLeft.setBoolean(false);
    }

    if (currentHorizontalLocation == HorizontalLocations.CENTER
        && currentVerticalLocation == VerticalLocations.MID) {
      middleMid.setBoolean(true);
    } else {
      middleMid.setBoolean(false);
    }

    if (currentHorizontalLocation == HorizontalLocations.RIGHT
        && currentVerticalLocation == VerticalLocations.MID) {
      middleRight.setBoolean(true);
    } else {
      middleRight.setBoolean(false);
    }

    if (currentHorizontalLocation == HorizontalLocations.LEFT
        && currentVerticalLocation == VerticalLocations.HIGH) {
      highLeft.setBoolean(true);
    } else {
      highLeft.setBoolean(false);
    }

    if (currentHorizontalLocation == HorizontalLocations.CENTER
        && currentVerticalLocation == VerticalLocations.HIGH) {
      highMiddle.setBoolean(true);
    } else {
      highMiddle.setBoolean(false);
    }

    if (currentHorizontalLocation == HorizontalLocations.RIGHT
        && currentVerticalLocation == VerticalLocations.HIGH) {
      highRight.setBoolean(true);
    } else {
      highRight.setBoolean(false);
    }

    GamePieceMode currentGamePiece = stateHandler.getGamePieceMode();
    if (currentGamePiece == GamePieceMode.CONE) {
      desiredGamePiece.setBoolean(true);
    } else {
      desiredGamePiece.setBoolean(false);
    }

    desiredArmPosition.setString(stateHandler.getArmDesiredPosition().toString());
    currentArmPosition.setString(stateHandler.getCurrentArmPosition().toString());

    desiredIntakePosition.setString(stateHandler.getDesiredIntakePosition().toString());
    currentIntakePosition.setString(stateHandler.getCurrentIntakePosition().toString());

    currentRobotDirection.setString(stateHandler.getRobotDirection().toString());

  }
}
