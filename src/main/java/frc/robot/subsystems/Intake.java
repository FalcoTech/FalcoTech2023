// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  private final CANSparkMax leftIntakeMotor = new CANSparkMax(IntakeConstants.INTAKELEFTMOTOR_ID, MotorType.kBrushless);
  private final CANSparkMax rightIntakeMotor = new CANSparkMax(IntakeConstants.INTAKERIGHTMOTOR_ID, MotorType.kBrushless);
  
  /** Creates a new Intake. */
  public Intake() {
    //leftIntakeMotor.follow(rightIntakeMotor, true);
    rightIntakeMotor.follow(leftIntakeMotor, true);
    leftIntakeMotor.setIdleMode(IdleMode.kBrake);
    rightIntakeMotor.setIdleMode(IdleMode.kBrake);
  }

  public void RunIntake(double speed){
    leftIntakeMotor.set(speed);
    //rightIntakeMotor.set(speed);
  }
  public void StopIntake(){
    leftIntakeMotor.set(0);
    rightIntakeMotor.set(0);
  }

  public double GetLeftIntakeMotorVoltage(){
    return leftIntakeMotor.getOutputCurrent();
  }
  public double GetRightIntakeMotorVoltage(){
    return rightIntakeMotor.getOutputCurrent();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Intake Motor Voltage", GetLeftIntakeMotorVoltage());
    SmartDashboard.putNumber("Right Intake Motor Voltage", GetRightIntakeMotorVoltage());
  }
}
