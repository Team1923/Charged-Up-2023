// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.interfaces;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.GamePieceColorConstants;
import frc.robot.interfaces.MKIPicoColorSensor.RawColor;

public class ColorSensorInterface extends SubsystemBase {

  private static ColorSensorInterface color;

  private MKIPicoColorSensor pico;

  private RawColor rColor0 = new RawColor();

  double red = 0;
  double blue = 0;
  double green = 0;

  public enum GamePiece{
    YELLOW_CONE,
    PURPLE_CUBE,
    UNKNOWN
  }
  
  public static synchronized ColorSensorInterface getInstance() {
    if (color == null) {
      color = new ColorSensorInterface(new MKIPicoColorSensor());
    }
    return color;
  }
  /** Creates a new ColorSensorInterface. */
  public ColorSensorInterface(MKIPicoColorSensor p) {
    this.pico = p;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    rColor0  =  pico.getRawColor0();
    red = GetRed();
    blue = GetBlue();
    green = GetGreen();  

  }

  //Ignoring red comparison 
  public boolean isCube(){  //purple cube is (151,23,160), 
 
    return (blue/green < Constants.GamePieceColorConstants.UNKNOWN_THRESHOLD - Constants.GamePieceColorConstants.Offset);
  }

  public boolean isCone(){ // yellow cone is (177,166,12)
    return (blue/green > Constants.GamePieceColorConstants.UNKNOWN_THRESHOLD + Constants.GamePieceColorConstants.Offset);
  }

  public String RGBValues(){

    return "Red: " +GetRed()+" Blue: "+ GetBlue()+" Green: "+GetGreen();

  }

  public double GetRed(){
    return rColor0.red;
  }
  public double GetBlue(){
    return rColor0.blue;
  }
  public double GetGreen(){
    return rColor0.green;
  }

  public GamePiece getGamePiece(){

    //all things realated to color sensor 1 commented since testing was done with only one color sensor

    //RawColor rcolor1 = pico.getRawColor1();
 
    // int red = (rcolor0.red + rcolor1.red)/2;
    // int blue = (rcolor0.blue + rcolor1.blue)/2;
    // int green = (rcolor0.green + rcolor1.green)/2;  


    // implement ir break sensor if logic later

    // if(!BeamBreakInterface.getInstance().isBroken()){
    //   return 0;
    // }
    /*else*/


    if(isCube()) {

      return GamePiece.PURPLE_CUBE;

    }
    else if (isCone()) {
      return GamePiece.YELLOW_CONE;
    }
   else {
    
      return GamePiece.UNKNOWN;
   }
 }
}

