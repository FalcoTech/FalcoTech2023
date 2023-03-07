// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.sql.Time;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.LEDsConstants;
import frc.robot.commands.*;


public class LEDs extends SubsystemBase {
  private final PWMSparkMax blinkin = new PWMSparkMax(9);

  /** Creates a new LEDs. */
  public LEDs() {
    Rainbow(); //start in rainbow
  }
  
  public void Rainbow(){
    blinkin.set(-.97); //RAINBOW: .99 rainbow palette, .97 party palette (PARTY MMODEDE YEAEAEAAAAHW WOAOOWOWOOOOOOOOOOOOOOOOOOOOOO ABABY LET'SD GOOOIIOOOO)
  }

  public void Purple(){
    blinkin.set(.91);
  }
  public void Yellow(){
    blinkin.set(.69);
  }

  public void Green(){
    blinkin.set(.73);
  }
  public void BlinkGreen(){
    blinkin.set(.05); //COLOR 1: .03 slow, .05 medium, .07 fast. 
  }
  public void Red(){
    blinkin.set(.61);
  }
  public void BlinkRed(){
    blinkin.set(-.25); //COLOR 2: .23 slow, .25 medium, .27 fast
  }

  public void SwitchHPColor(){
    if (blinkin.get() == -.97){
      Purple();
    } else if (blinkin.get() == .91){
      Yellow();
    } else if (blinkin.get() == .69){
      Purple();
    }
    
  }


  @Override
  public void periodic() {// This method will be called once per scheduler run
    
  }
}
