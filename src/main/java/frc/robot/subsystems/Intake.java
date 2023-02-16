// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  // private final CANSparkMax intakeLeftMotor = new CANSparkMax(IntakeConstants.INTAKELEFTMOTOR_ID, MotorType.kBrushless);
  // private final CANSparkMax intakeRightMotor = new CANSparkMax(IntakeConstants.INTAKERIGHTMOTOR_ID, MotorType.kBrushless);

  // public final ColorSensorV3 intakeColorSensor = new ColorSensorV3(I2C.Port.kOnboard);
  
  /** Creates a new Intake. */
  public Intake() {
    // intakeRightMotor.follow(intakeLeftMotor, true);
  }

  
  // public void runIntake(double speed){
  //    intakeLeftMotor.set(speed);
  // }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
