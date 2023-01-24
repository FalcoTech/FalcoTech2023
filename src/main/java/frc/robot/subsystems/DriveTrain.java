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
  private final CANSparkMax leftFrontMotor = new CANSparkMax(DriveTrainConstants.leftFrontMotor_ID, MotorType.kBrushless);
  private final CANSparkMax leftBackMotor = new CANSparkMax(DriveTrainConstants.leftBackMotor_ID, MotorType.kBrushless); 
  private final CANSparkMax rightFrontMotor = new CANSparkMax(DriveTrainConstants.rightFrontMotor_ID, MotorType.kBrushless); 
  private final CANSparkMax rightBackMotor = new CANSparkMax(DriveTrainConstants.rightBackMotor_ID, MotorType.kBrushless); 

  //Differentialdrive Inits
  private final MotorControllerGroup leftDriveGroup = new MotorControllerGroup(leftFrontMotor, leftBackMotor);
  private final MotorControllerGroup rightDriveGroup = new MotorControllerGroup(rightFrontMotor, rightBackMotor);
  private final DifferentialDrive m_Drive = new DifferentialDrive(leftDriveGroup, rightDriveGroup);

  ///Encoders 
  private final RelativeEncoder leftDriveEncoder = leftFrontMotor.getEncoder();
  private final RelativeEncoder rightDriveEncoder = rightFrontMotor.getEncoder();

  //Gyro
  private final ADIS16470_IMU gyro = new ADIS16470_IMU();

  //Compressor/Solenoids Inits
  private final Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);
  private final DoubleSolenoid shiftSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, DriveTrainConstants.shiftSolForward_ID, DriveTrainConstants.shiftSolReverse_ID);

  //Booleans/Strings
  public String arcadeDriveSpeed = "default"; 


  /** Creates a new DriveTrain. */
  public DriveTrain() {
    leftDriveGroup.setInverted(true); //Invert one side drive motors

    shiftSolenoid.set(Value.kForward); //Start in low gear
    coastDriveMotors(); //Start coasting drive motors
    arcadeDriveSpeed = "default"; //regular speed
    
    //Motor follows
    leftBackMotor.follow(leftFrontMotor);
    rightBackMotor.follow(rightFrontMotor);

  }

  //Our main ArcadeDrive command. 
  public void arcadeDrive(double speed, double rotation){
    m_Drive.arcadeDrive(speed, -rotation, false);
  } 
  //Secondary ArcadeDrive command. Has additional bool for squared inputs to increase controlability at low speeds. 
  public void arcadeDrive(double speed, double rotation, boolean isSquaredInputs){
    m_Drive.arcadeDrive(speed, -rotation, isSquaredInputs);
  } 

  public void shiftLowGear(){
    shiftSolenoid.set(Value.kForward);
  }
  public void shiftHighGear(){
    shiftSolenoid.set(Value.kReverse);
  }

  public void brakeDriveMotors(){
    leftFrontMotor.setIdleMode(IdleMode.kBrake);
    leftBackMotor.setIdleMode(IdleMode.kBrake);
    rightFrontMotor.setIdleMode(IdleMode.kBrake);
    rightBackMotor.setIdleMode(IdleMode.kBrake);
  }
  public void coastDriveMotors(){
    leftFrontMotor.setIdleMode(IdleMode.kCoast);
    leftBackMotor.setIdleMode(IdleMode.kCoast);
    rightFrontMotor.setIdleMode(IdleMode.kCoast);
    rightBackMotor.setIdleMode(IdleMode.kCoast);
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
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    compressor.enableDigital(); //Runs compressor
  }
}
