// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.interfaces;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BeamBreakInterface extends SubsystemBase {
private DigitalInput beamBreak1 = new DigitalInput(Constants.DigitalIDConstants.DigitalID);//?????

private static BeamBreakInterface beamBreak;


public static synchronized BeamBreakInterface getInstance() {
  if (beamBreak == null) {
    beamBreak = new BeamBreakInterface();
  }
  return beamBreak;
}
  /** Creates a new BeamBreakInterface. */
  public BeamBreakInterface() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public boolean isBroken(){// dont know id
    return !beamBreak1.get();
  }

}

