// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.beans.Encoder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.DriveTrainConstants;


public class DriveTrain extends SubsystemBase {
  private CANSparkMax leftFrontMotor = new CANSparkMax(DriveTrainConstants.leftFrontMotor_ID, MotorType.kBrushless);
  private CANSparkMax leftBackMotor = new CANSparkMax(DriveTrainConstants.leftBackMotor_ID, MotorType.kBrushless); 
  private CANSparkMax rightFrontMotor = new CANSparkMax(DriveTrainConstants.rightFrontMotor_ID, MotorType.kBrushless); 
  private CANSparkMax rightBackMotor = new CANSparkMax(DriveTrainConstants.rightBackMotor_ID, MotorType.kBrushless);  

  private SparkMaxPIDController leftPID = leftFrontMotor.getPIDController();
  private SparkMaxPIDController rightPID = rightFrontMotor.getPIDController();

  private final MotorControllerGroup m_leftdrive = new MotorControllerGroup(leftFrontMotor, leftBackMotor);
  private final MotorControllerGroup m_rightdrive = new MotorControllerGroup(rightFrontMotor, rightBackMotor);

  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftdrive, m_rightdrive);
  
  /** Creates a new DriveTrain. */
  public DriveTrain() {
    m_rightdrive.setInverted(true);

    leftFrontMotor.setIdleMode(IdleMode.kCoast);
    leftBackMotor.setIdleMode(IdleMode.kCoast);
    rightFrontMotor.setIdleMode(IdleMode.kCoast);
    rightBackMotor.setIdleMode(IdleMode.kCoast);

    leftBackMotor.follow(leftFrontMotor);
    rightBackMotor.follow(rightFrontMotor);
  }


  public void arcadeDrive(double speed, double rotation){
    m_drive.arcadeDrive(speed, rotation, false);
  } 
  public void arcadeDrive(double speed, double rotation, boolean isSquaredInputs){
    m_drive.arcadeDrive(speed, rotation, isSquaredInputs);
  } 


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
