// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.LEDsConstants;

public class LEDs extends SubsystemBase {
  private final AddressableLED LEDStripLeft = new AddressableLED(LEDsConstants.LEDSTRIPLEFTPORT);
  private final AddressableLED LEDStripRight = new AddressableLED(LEDsConstants.LEDSTRIPRIGHTPORT);
  
  private final AddressableLEDBuffer LEDStripBuffer = new AddressableLEDBuffer(LEDsConstants.LEDSTRIPLENGTH);

  private String LightsColor = "";


  /** Creates a new LEDs. */
  public LEDs() {
    LEDStripLeft.setLength(LEDStripBuffer.getLength());
    LEDStripRight.setLength(LEDStripBuffer.getLength());
    
    LEDStripLeft.setData(LEDStripBuffer);
    LEDStripRight.setData(LEDStripBuffer);
    
    LEDStripLeft.start();
    LEDStripRight.start();
  }

  public void ChangeLEDColor(int red, int green, int blue){
    for (var i = 0; i < LEDStripBuffer.getLength(); i++){
      LEDStripBuffer.setRGB(i, red, green, blue);
    } 
  }
  
  public void HumanPlayerLEDSwitch(){
    if (LightsColor != "Yellow"){
      ChangeLEDColor(255, 255, 0);
      LightsColor = "Yellow";
    } else{
      ChangeLEDColor(255, 0, 255);
      LightsColor = "Purple";
    }
  }

  @Override
  public void periodic() {// This method will be called once per scheduler run
    LEDStripLeft.setData(LEDStripBuffer);
    LEDStripRight.setData(LEDStripBuffer);
  }
}
