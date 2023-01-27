// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.interfaces;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.MKILib.MKIPicoColorSensor;
import frc.robot.MKILib.MKIPicoColorSensor.RawColor;

public class ColorSensorInterface extends SubsystemBase {

  private static ColorSensorInterface color;

  private static MKIPicoColorSensor pico;
  
  public static synchronized ColorSensorInterface getInstance() {
    if (color == null) {
      color = new ColorSensorInterface();
    }
    return color;
  }
  /** Creates a new ColorSensorInterface. */
  public ColorSensorInterface() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //Ignoring red comparison 
  public boolean isCube(int r, int g, int b){  //purple cube is (151,23,160), 
 
   if(Math.abs(g - Constants.GamePieceColorConstants.CUBEGREEN) <= 25 && Math.abs(b - Constants.GamePieceColorConstants.CUBEBLUE) <= 25){
    return true;
   }
   else{
    return false;
   }
  }

  public boolean isCone(int r, int g, int b){ // yellow cone is (177,166,12)
    if(Math.abs(g - Constants.GamePieceColorConstants.CONEGREEN) <= 25 && Math.abs(b - Constants.GamePieceColorConstants.CONEBLUE) <= 25){
      return true;
     }
     else{
      return false;
     }
  }

  public int getGamePiece(){
    RawColor rcolor0  =  pico.getRawColor0();
    RawColor rcolor1 = pico.getRawColor1();
 
    int red = (rcolor0.red + rcolor1.red)/2;
    int blue = (rcolor0.blue + rcolor1.blue)/2;
    int green = (rcolor0.green + rcolor1.green)/2;

    // implement ir break sensor if logic later

    if(!BeamBreakInterface.getInstance().isBroken()){
      return 0;
    }
    else if(isCube(red, green, blue)) {

      return 1;

    }
    else if (isCone(red, green, blue)) {
      return -1;
    }
   return 0;
  }
}

