// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  private final CANSparkMax intakeLeftMotor = new CANSparkMax(IntakeConstants.intakeLeftMotor_ID, MotorType.kBrushless);
  private final CANSparkMax intakeRightMotor = new CANSparkMax(IntakeConstants.intakeRightMotor_ID, MotorType.kBrushless);

  /** Creates a new Intake. */
  public Intake() {

    
  }

  public void runIntake(double speed){
    intakeLeftMotor.set(speed);
    intakeRightMotor.set(speed);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
