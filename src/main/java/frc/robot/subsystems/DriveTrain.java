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

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.DriveTrainConstants;


public class DriveTrain extends SubsystemBase {
  //Motor Inits
  private CANSparkMax leftFrontMotor = new CANSparkMax(DriveTrainConstants.leftFrontMotor_ID, MotorType.kBrushless);
  private CANSparkMax leftBackMotor = new CANSparkMax(DriveTrainConstants.leftBackMotor_ID, MotorType.kBrushless); 
  private CANSparkMax rightFrontMotor = new CANSparkMax(DriveTrainConstants.rightFrontMotor_ID, MotorType.kBrushless); 
  private CANSparkMax rightBackMotor = new CANSparkMax(DriveTrainConstants.rightBackMotor_ID, MotorType.kBrushless); 
  //PID Inits
  private SparkMaxPIDController leftPID = leftFrontMotor.getPIDController();
  private SparkMaxPIDController rightPID = rightFrontMotor.getPIDController();
  //Differentialdrive Inits
  private final MotorControllerGroup m_leftdrive = new MotorControllerGroup(leftFrontMotor, leftBackMotor);
  private final MotorControllerGroup m_rightdrive = new MotorControllerGroup(rightFrontMotor, rightBackMotor);
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftdrive, m_rightdrive);
  //Compressor/Solenoids Inits
  private final Compressor phCompressor = new Compressor(PneumaticsModuleType.REVPH);
  private final DoubleSolenoid shiftSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, DriveTrainConstants.shiftSolForward_ID, DriveTrainConstants.shiftSolReverse_ID);
  //Booleans/Strings
  public String arcadeDriveSpeed = "default"; 
  
  /** Creates a new DriveTrain. */
  public DriveTrain() {
    //Sets one drive train side to inverted
    m_leftdrive.setInverted(true); //check this

    //Coast motors and set default speed 
    leftFrontMotor.setIdleMode(IdleMode.kCoast);
    leftBackMotor.setIdleMode(IdleMode.kCoast);
    rightFrontMotor.setIdleMode(IdleMode.kCoast);
    rightBackMotor.setIdleMode(IdleMode.kCoast);
    arcadeDriveSpeed = "default";

    //Back motors will always follow the lead motors
    leftBackMotor.follow(leftFrontMotor);
    rightBackMotor.follow(rightFrontMotor);

    //Shift solenoid defaults to low gear
    shiftSolenoid.set(Value.kForward);
  }

  //Our main ArcadeDrive command. 
  public void arcadeDrive(double speed, double rotation){
    m_drive.arcadeDrive(speed, rotation, false);
  } 
  //Secondary ArcadeDrive command. Has additional bool for squared inputs to increase controlability at low speeds. 
  public void arcadeDrive(double speed, double rotation, boolean isSquaredInputs){
    m_drive.arcadeDrive(speed, rotation, isSquaredInputs);
  } 

  public void shiftLowGear(){
    shiftSolenoid.set(Value.kForward);
  }
  
  public void shiftHighGear(){
    shiftSolenoid.set(Value.kReverse);
  }

  public void toggleGear(){
    shiftSolenoid.toggle();
  }

  public void toggleArcadeDriveSpeed(){
    if (arcadeDriveSpeed == "default"){
      arcadeDriveSpeed = "slow";
      leftFrontMotor.setIdleMode(IdleMode.kBrake);
      leftBackMotor.setIdleMode(IdleMode.kBrake);
      rightFrontMotor.setIdleMode(IdleMode.kBrake);
      rightBackMotor.setIdleMode(IdleMode.kBrake);

    } else{
      arcadeDriveSpeed = "default";
      leftFrontMotor.setIdleMode(IdleMode.kCoast);
      leftBackMotor.setIdleMode(IdleMode.kCoast);
      rightFrontMotor.setIdleMode(IdleMode.kCoast);
      rightBackMotor.setIdleMode(IdleMode.kCoast);
      
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    phCompressor.enableDigital(); //Runs compressor
  }
}
