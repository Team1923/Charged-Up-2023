// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.StateHandler;

public class ControllerRumble extends SubsystemBase {
  /** Creates a new ControllerRumble. */
  private XboxController xboxController;
  private StateHandler stateHandler = StateHandler.getInstance();

  public ControllerRumble(XboxController xboxController) {
    this.xboxController = xboxController;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(DriverStation.isTeleop() && stateHandler.getGamePiece()) {
      xboxController.setRumble(RumbleType.kBothRumble, 0.75);
    } else {
      xboxController.setRumble(RumbleType.kBothRumble, 0);
    }
  }
}
