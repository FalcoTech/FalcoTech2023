// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  private final CANSparkMax leftIntakeMotor = new CANSparkMax(IntakeConstants.INTAKELEFTMOTOR_ID, MotorType.kBrushless);
  private final CANSparkMax rightIntakeMotor = new CANSparkMax(IntakeConstants.INTAKERIGHTMOTOR_ID, MotorType.kBrushless);
  
  /** Creates a new Intake. */
  public Intake() {
    rightIntakeMotor.follow(leftIntakeMotor, false); //change "false" to "true" if one set of motors need to be inverted. 
  }

  public void RunIntake(double speed){
    leftIntakeMotor.set(speed);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
