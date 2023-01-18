// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.beans.Encoder;

import javax.swing.text.Position;

import com.pathplanner.lib.auto.BaseAutoBuilder;
import com.pathplanner.lib.auto.RamseteAutoBuilder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.OperatorConstants;


public class DriveTrain extends SubsystemBase {
  //Motor Inits
  private CANSparkMax m_leftFrontMotor = new CANSparkMax(DriveTrainConstants.leftFrontMotor_ID, MotorType.kBrushless);
  private CANSparkMax m_leftBackMotor = new CANSparkMax(DriveTrainConstants.leftBackMotor_ID, MotorType.kBrushless); 
  private CANSparkMax m_rightFrontMotor = new CANSparkMax(DriveTrainConstants.rightFrontMotor_ID, MotorType.kBrushless); 
  private CANSparkMax m_rightBackMotor = new CANSparkMax(DriveTrainConstants.rightBackMotor_ID, MotorType.kBrushless); 

  //Differentialdrive Inits
  private final MotorControllerGroup m_leftDrive = new MotorControllerGroup(m_leftFrontMotor, m_leftBackMotor);
  private final MotorControllerGroup m_rightDrive = new MotorControllerGroup(m_rightFrontMotor, m_rightBackMotor);
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftDrive, m_rightDrive);

  //Gyro
  private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();

  //Encoder Inits (Check this, RelativeEncoder may not be the right statement)
  private final RelativeEncoder m_leftDriveEncoder = m_leftFrontMotor.getEncoder();
  private final RelativeEncoder m_rightDriveEncoder = m_rightFrontMotor.getEncoder();

  //DifferentialDrive Odometry
  private final DifferentialDriveOdometry m_odometry;

  //Compressor/Solenoids Inits
  private final Compressor phCompressor = new Compressor(PneumaticsModuleType.REVPH);
  private final DoubleSolenoid shiftSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, DriveTrainConstants.shiftSolForward_ID, DriveTrainConstants.shiftSolReverse_ID);
  //Booleans/Strings
  public String arcadeDriveSpeed = "default"; 
  
  /** Creates a new DriveTrain. */
  public DriveTrain() {
    m_leftDrive.setInverted(true); //Sets one drive train side to inverted. (This is correct now)
  
    m_gyro.reset();
    resetOdometry();//Resets encoder and gyro values
    m_odometry = new DifferentialDriveOdometry(getRotation2d(), 0, 0);

    //Coast motors and set default speed 
    coastDriveMotors();
    arcadeDriveSpeed = "default";

    //Back motors will always follow the lead motors
    m_leftBackMotor.follow(m_leftFrontMotor);
    m_rightBackMotor.follow(m_rightFrontMotor);

    //Shift solenoid defaults to low gear
    shiftSolenoid.set(Value.kForward);
    
    
  }

  //Our main ArcadeDrive command. 
  public void arcadeDrive(double speed, double rotation){
    m_drive.arcadeDrive(speed, -rotation, false);
  } 
  //Secondary ArcadeDrive command. Has additional bool for squared inputs to increase controlability at low speeds. 
  public void arcadeDrive(double speed, double rotation, boolean isSquaredInputs){
    m_drive.arcadeDrive(speed, -rotation, isSquaredInputs);
  } 

  public void shiftLowGear(){
    shiftSolenoid.set(Value.kForward);
  }
  
  public void shiftHighGear(){
    shiftSolenoid.set(Value.kReverse);
  }

  public void toggleArcadeDriveSpeed(){
    if (arcadeDriveSpeed == "default"){
      arcadeDriveSpeed = "slow";
      brakeDriveMotors();

    } else{
      arcadeDriveSpeed = "default";
      coastDriveMotors();
    }
  }

  public void brakeDriveMotors(){
    m_leftFrontMotor.setIdleMode(IdleMode.kBrake);
    m_leftBackMotor.setIdleMode(IdleMode.kBrake);
    m_rightFrontMotor.setIdleMode(IdleMode.kBrake);
    m_rightBackMotor.setIdleMode(IdleMode.kBrake);
  }

  public void coastDriveMotors(){
    m_leftFrontMotor.setIdleMode(IdleMode.kCoast);
    m_leftBackMotor.setIdleMode(IdleMode.kCoast);
    m_rightFrontMotor.setIdleMode(IdleMode.kCoast);
    m_rightBackMotor.setIdleMode(IdleMode.kCoast);
  }

  public Rotation2d getRotation2d(){
    double gyroAngle = m_gyro.getAngle();
    return Rotation2d.fromDegrees(gyroAngle);
  }

  public Pose2d getPose2d(){
    return m_odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds(
      m_leftFrontMotor.getEncoder().getVelocity(), 
      m_rightFrontMotor.getEncoder().getVelocity());
  }

  public void updateOdometry(){
    m_odometry.update(getRotation2d(), 
      m_leftFrontMotor.getEncoder().getPosition(), 
      m_rightFrontMotor.getEncoder().getPosition());
  }

  public void resetOdometry(){
    m_leftDriveEncoder.setPosition(0);
    m_rightDriveEncoder.setPosition(0);
    m_gyro.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    phCompressor.enableDigital(); //Runs compressor
    updateOdometry(); //Updates odometry values
  }
}
