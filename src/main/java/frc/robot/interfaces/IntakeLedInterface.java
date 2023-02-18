// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.interfaces;

import java.util.ArrayList;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeLedInterface extends SubsystemBase {

  private static IntakeLedInterface ledInterface;
  private int currentRed = 0; 
  private boolean getBrighter = true;

  private static AddressableLED led;
  private static AddressableLEDBuffer ledbuffer = new AddressableLEDBuffer(100);

  private static int rainbowFirstPixelHue = 0;


  public static synchronized IntakeLedInterface getInstance() {
    if (ledInterface == null) {
      ledInterface = new IntakeLedInterface();
    }
    return ledInterface;
  }

  /** Creates a new LedInterface. */
  public IntakeLedInterface() {
    led = new AddressableLED(9);
    led.setLength(ledbuffer.getLength());
    led.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setGreen() {
    for(int i = 0; i < ledbuffer.getLength(); i ++){
      ledbuffer.setRGB(i,0,255,0);
    }
    led.setData(ledbuffer);
  }

  public void setRed(){
    for(int i = 0; i < ledbuffer.getLength(); i ++){
      ledbuffer.setRGB(i, 255, 0, 0);
    }
    led.setData(ledbuffer);
  }

  public void setYellow(){
    for(int i = 0; i < ledbuffer.getLength(); i++){
      ledbuffer.setRGB(i, 255, 255, 0);
    }
    led.setData(ledbuffer);
  }

  public void setPurple(){
    for(int i = 0; i < ledbuffer.getLength(); i++) {
      ledbuffer.setRGB(i, 119, 0, 200);
    }
    led.setData(ledbuffer);
  }

  public void setBlank() {
    for(int i = 0; i < ledbuffer.getLength(); i++) {
      ledbuffer.setRGB(i, 0, 0, 0);
    }
    led.setData(ledbuffer);
  }

  public void setIndividualLed(ArrayList<Color> a) {
    for(int i = 0; i < ledbuffer.getLength(); i++) {
      ledbuffer.setLED(i, a.get(i));
    }
    led.setData(ledbuffer);
  }

  public void rainbow(){//Works
    for(var i = 0; i < ledbuffer.getLength(); i++){
      final var hue = (rainbowFirstPixelHue + (i * 180/ledbuffer.getLength())) % 180;

      ledbuffer.setHSV(i, hue, 255, 128);
    }
    rainbowFirstPixelHue += 3;
    rainbowFirstPixelHue %= 180;

    led.setData(ledbuffer);
  }

  public void redOsciliating(){
    if(getBrighter){
      currentRed +=15;
    }
    else{
      currentRed -=15;
    }

    for(int i = 0; i < ledbuffer.getLength(); i ++){
      ledbuffer.setRGB(i, currentRed, 0, 0);
      if(currentRed >= 255){
        currentRed = 255;
        getBrighter = false;
      }
      else if(currentRed <= 0){
        currentRed = 0; 
        getBrighter = true;
      }
    }
    led.setData(ledbuffer);
  }

}
