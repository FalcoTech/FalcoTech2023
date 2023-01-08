// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveTrainConstants;

public class DriveTrainSub extends SubsystemBase {
  private static final CANSparkMax leftFrontMotor = new CANSparkMax(DriveTrainConstants.leftFrontMotor_ID, MotorType.kBrushless);
  private static final CANSparkMax leftBackMotor = new CANSparkMax(DriveTrainConstants.leftBackMotor_ID, MotorType.kBrushless); 
  private static final CANSparkMax rightFrontMotor = new CANSparkMax(DriveTrainConstants.rightFrontMotor_ID, MotorType.kBrushless); 
  private static final CANSparkMax rightBackMotor = new CANSparkMax(DriveTrainConstants.rightBackMotor_ID, MotorType.kBrushless);  

  public static final MotorControllerGroup m_leftdrive = new MotorControllerGroup(leftFrontMotor, leftBackMotor);
  public static final MotorControllerGroup m_rightdrive = new MotorControllerGroup(rightFrontMotor, rightBackMotor);

  public static final DifferentialDrive m_drive = new DifferentialDrive(m_leftdrive, m_rightdrive);


  /** Creates a new DriveTrain. */
  public DriveTrainSub() {
    m_rightdrive.setInverted(true);

    leftFrontMotor.setIdleMode(IdleMode.kCoast);
    leftBackMotor.setIdleMode(IdleMode.kCoast);
    rightFrontMotor.setIdleMode(IdleMode.kCoast);
    rightBackMotor.setIdleMode(IdleMode.kCoast);

  }

  public static void arcadeDrive(double leftY, double rightX){
    m_drive.arcadeDrive(leftY, rightX, false);
  } 

  public static void arcadeDrive(double leftY, double rightX, boolean isSquareInputs){
    m_drive.arcadeDrive(leftY, rightX, isSquareInputs);
  } 


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
