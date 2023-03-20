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
import frc.robot.util.StateVariables.CurrentRobotDirection;
import frc.robot.util.StateVariables.GamePieceMode;
import frc.robot.util.StateVariables.HorizontalLocations;
import frc.robot.util.StateVariables.IntakePositions;
import frc.robot.util.StateVariables.VerticalLocations;

public class ShuffleboardSubsystem extends SubsystemBase {
	private StateHandler stateHandler = StateHandler.getInstance();

	public ShuffleboardTab driverDashboard = Shuffleboard.getTab("Driver Dashboard");

	private GenericEntry high = driverDashboard.add("HIGH", false)
			.withSize(2, 1)
			.withPosition(0, 0)
			.withProperties(Map.of("Color when false", "#000000", "Color when true", "#17FC03"))
			.getEntry();

	private GenericEntry mid = driverDashboard.add("MID", false)
			.withSize(2, 1)
			.withPosition(0, 1)
			.withProperties(Map.of("Color when false", "#000000", "Color when true", "#17FC03"))
			.getEntry();

	private GenericEntry low = driverDashboard.add("LOW", false)
			.withSize(2, 1)
			.withPosition(0, 2)
			.withProperties(Map.of("Color when false", "#000000", "Color when true", "#17FC03"))
			.getEntry();

	private GenericEntry desiredGamePiece = driverDashboard.add("Game Piece", false)
			.withSize(2, 1)
			.withPosition(2, 0)
			.withProperties(Map.of("Color when false", "#7842f5", "Color when true", "#f5ef42"))
			.getEntry();

	private GenericEntry desiredIntakePosition = driverDashboard
			.add("DESIRED INTAKE", IntakePositions.STOW.toString())
			.withSize(2, 1)
			.withPosition(2, 2)
			.getEntry();



	@Override
	public void periodic() {
		VerticalLocations currentVerticalLocation = stateHandler.getCurrentVerticalLocation();

		if (currentVerticalLocation == VerticalLocations.HIGH) {
			high.setBoolean(true);
		} else {
			high.setBoolean(false);
		}

		if (currentVerticalLocation == VerticalLocations.MID) {
			mid.setBoolean(true);
		} else {
			mid.setBoolean(false);
		}

		if (currentVerticalLocation == VerticalLocations.LOW) {
			low.setBoolean(true);
		} else {
			low.setBoolean(false);
		}

		GamePieceMode currentGamePiece = stateHandler.getGamePieceMode();
		if (currentGamePiece == GamePieceMode.CONE) {
			desiredGamePiece.setBoolean(true);
		} else {
			desiredGamePiece.setBoolean(false);
		}

		desiredIntakePosition.setString(stateHandler.getDesiredIntakePosition().toString());

	}
}
