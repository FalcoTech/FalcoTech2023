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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  ///Encoders 
  private final RelativeEncoder m_leftDriveEncoder = m_leftFrontMotor.getEncoder();
  private final RelativeEncoder m_rightDriveEncoder = m_rightFrontMotor.getEncoder();

  //Gyro
  private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();

  //Odometry
  private final DifferentialDriveOdometry m_odometry;

  //Compressor/Solenoids Inits
  private final Compressor m_compressor = new Compressor(PneumaticsModuleType.REVPH);
  private final DoubleSolenoid shiftSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, DriveTrainConstants.shiftSolForward_ID, DriveTrainConstants.shiftSolReverse_ID);

  //Field Map
  private final Field2d m_field = new Field2d();

  //Booleans/Strings
  public String arcadeDriveSpeed = "default"; 


  /** Creates a new DriveTrain. */
  public DriveTrain() {
    m_leftDrive.setInverted(true); //Sets one drive train side to inverted. (This is correct now)

    //Coast motors and set default speed 
    coastDriveMotors();
    arcadeDriveSpeed = "default";
    shiftSolenoid.set(Value.kForward); //Shift solenoid defaults to low gear

    //Back motors will always follow the lead motors
    m_leftBackMotor.follow(m_leftFrontMotor);
    m_rightBackMotor.follow(m_rightFrontMotor);

    //Define odometry
    resetDriveEncoders();
    m_odometry = new DifferentialDriveOdometry(getRotation2d(), getRightDriveEncoderDist(), getLeftDriveEncoderDist());

    //Add map to smartdashboard
    SmartDashboard.putData("Field", m_field);
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

  public double getLeftDriveEncoderDist(){
    return m_leftDriveEncoder.getPosition() / DriveTrainConstants.driveTicksPerRevolution * DriveTrainConstants.driveGearRatio * DriveTrainConstants.driveDistPerRev;
  }
  public double getRightDriveEncoderDist(){
    return m_rightDriveEncoder.getPosition() / DriveTrainConstants.driveTicksPerRevolution * DriveTrainConstants.driveGearRatio * DriveTrainConstants.driveDistPerRev;
  }
  public void resetDriveEncoders(){
    m_leftDriveEncoder.setPosition(0);
    m_rightDriveEncoder.setPosition(0);
  }

  public Rotation2d getRotation2d(){
    return Rotation2d.fromDegrees(m_gyro.getAngle());
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
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_compressor.enableDigital(); //Runs compressor
    getRotation2d();
    getLeftDriveEncoderDist();
    getRightDriveEncoderDist();

    m_odometry.update(getRotation2d(), getLeftDriveEncoderDist(), getRightDriveEncoderDist());
  }
}
