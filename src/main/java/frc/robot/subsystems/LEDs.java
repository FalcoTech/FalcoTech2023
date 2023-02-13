// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.LEDsConstants;

public class LEDs extends SubsystemBase {
  // private final AddressableLED LEDStripLeft = new AddressableLED(LEDsConstants.LEDSTRIPLEFTPORT);
  // private final AddressableLED LEDStripRight = new AddressableLED(LEDsConstants.LEDSTRIPRIGHTPORT);
  private final AddressableLED testLEDStrip = new AddressableLED(9);
  private final AddressableLEDBuffer LEDBuffer = new AddressableLEDBuffer(LEDsConstants.LEDSTRIPLENGTH);


  private int rainbowFirstPixelHue;
  private int purplePixelHue = 270; 

  /** Creates a new LEDs. */
  public LEDs() {
    testLEDStrip.setLength(LEDBuffer.getLength());
    testLEDStrip.setData(LEDBuffer);
    testLEDStrip.start();
  

  }

  public void ChangeLEDColorRGB(int red, int green, int blue){
    for (var i = 0; i < LEDBuffer.getLength(); i++){
      LEDBuffer.setRGB(i, red, green, blue);
    } 
    testLEDStrip.setData(LEDBuffer);
  }
  
  public void ChangeLEDColorHSV(int hue){
    for (var i = 0; i < LEDBuffer.getLength(); i++){
      LEDBuffer.setHSV(i, hue, 255, 128);;
    } 
    testLEDStrip.setData(LEDBuffer);
  }

  public void Rainbow(){
    for (var i = 0; i < LEDBuffer.getLength(); i++) {
      final var hue = (rainbowFirstPixelHue + (i * 180 / LEDBuffer.getLength())) % 180;
      LEDBuffer.setHSV(i, hue, 255, 128);
    }
    rainbowFirstPixelHue += 3;
    rainbowFirstPixelHue %= 180;
    testLEDStrip.setData(LEDBuffer);
  }


  @Override
  public void periodic() {// This method will be called once per scheduler run
    // LEDStripLeft.setData(LEDStripBuffer);
    // LEDStripRight.setData(LEDStripBuffer);
    // ChangeLEDColor(255, 0, 255); THIS breaks robot lol
    // Rainbow(); works goo
    testLEDStrip.setData(LEDBuffer);
  }
}
