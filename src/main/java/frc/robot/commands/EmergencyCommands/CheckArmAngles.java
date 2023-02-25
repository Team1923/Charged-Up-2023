// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.EmergencyCommands;

import frc.robot.Constants.ArmConstants;
import frc.robot.util.StateVariables.ArmPositions;

/** Add your docs here. */
public class CheckArmAngles {

	public static CheckArmAngles checkArmAngles;

	public static synchronized CheckArmAngles getInstance(){
		if(checkArmAngles == null) {
			checkArmAngles = new CheckArmAngles();
		}
		return checkArmAngles;
	}

	// cycle through the enums for Arm Angles
	public void checkIfInRange() {
		for (ArmPositions armPositions : ArmPositions.values()) {
			if (armPositions.getArmAngles().getProximalAngle() > ArmConstants.maxProximalPosition
					|| armPositions.getArmAngles().getProximalAngle() < ArmConstants.minProximalPosition) {
				throw new IllegalArgumentException(
						"PROXIMAL IS UNABLE TO GO TO: " + armPositions.toString() + " RIGHT");
			}

			if (armPositions.getArmAngles().getDistalAngle() > ArmConstants.maxDistalPosition
					|| armPositions.getArmAngles().getDistalAngle() < ArmConstants.minDistalPosition) {
				throw new IllegalArgumentException(
						"DISTAL IS UNABLE TO GO TO: " + armPositions.toString() + " RIGHT");
			}

			if (armPositions.getLeftArmAngles().getProximalAngle() > ArmConstants.maxProximalPosition
					|| armPositions.getLeftArmAngles().getProximalAngle() < ArmConstants.minProximalPosition) {
				throw new IllegalArgumentException(
						"PROXIMAL IS UNABLE TO GO TO: " + armPositions.toString() + " LEFT");
			}

			if (armPositions.getLeftArmAngles().getDistalAngle() > ArmConstants.maxDistalPosition
					|| armPositions.getLeftArmAngles().getDistalAngle() < ArmConstants.minDistalPosition) {
				throw new IllegalArgumentException(
						"DISTAL IS UNABLE TO GO TO: " + armPositions.toString() + " LEFT");
			}

		}
	}
}
