// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.sql.Time;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.LEDsConstants;
import frc.robot.commands.*;


public class LEDs extends SubsystemBase {
  private final Spark blinkin = new Spark(9);

  /** Creates a new LEDs. */
  public LEDs() {
    Rainbow(); //start in rainbow
  }
  
  public void Rainbow(){
    blinkin.set(LEDsConstants.RAINBOW_PARTYPAL); //RAINBOW: .99 rainbow palette, .97 party palette (PARTY MMODEDE YEAEAEAAAAHW WOAOOWOWOOOOOOOOOOOOOOOOOOOOOO ABABY LET'SD GOOOIIOOOO)
  }

  public void Purple(){
    blinkin.set(LEDsConstants.PURPLE);
  }
  public void Yellow(){
    blinkin.set(LEDsConstants.YELLOW);
  }

  public void Green(){
    blinkin.set(LEDsConstants.GREEN);
  }
  public void BlinkGreen(){
    blinkin.set(LEDsConstants.BLINKGREEN); //COLOR 1: .03 slow, .05 medium, .07 fast. 
  }
  public void Red(){
    blinkin.set(LEDsConstants.RED);
  }
  public void BlinkRed(){
    blinkin.set(LEDsConstants.BLINKRED); //COLOR 2: .23 slow, .25 medium, .27 fast
  }

  public void SwitchHPColor(){
    if (blinkin.get() == LEDsConstants.RAINBOW_RAINBOWPAL || blinkin.get() == LEDsConstants.RAINBOW_PARTYPAL){
      Purple();
    } else if (blinkin.get() == LEDsConstants.PURPLE){
      Yellow();
    } else if (blinkin.get() == LEDsConstants.YELLOW){
      Purple();
    } else{
      Purple();
    }
  }


  @Override
  public void periodic() {// This method will be called once per scheduler run
  }
}
