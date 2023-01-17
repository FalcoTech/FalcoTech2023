// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private final CANSparkMax m_intakeFrontMotor = new CANSparkMax(21, MotorType.kBrushless);
  private final CANSparkMax m_intakeBackMotor = new CANSparkMax(20, MotorType.kBrushless);

  public Intake() {

  }

  public void runIntake(){
    double PilotGetRightTrigger = RobotContainer.Pilot.getRightTriggerAxis();
    double PilotGetLeftTrigger = RobotContainer.Pilot.getLeftTriggerAxis();
    double intakePercent = (PilotGetRightTrigger - PilotGetLeftTrigger);

    m_intakeFrontMotor.set(intakePercent);
    m_intakeBackMotor.set(intakePercent);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
