// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveTrainConstants;


public class DriveTrain extends SubsystemBase {
  //Sparkmax motor controllers (old)
  private final CANSparkMax leftFrontMotor = new CANSparkMax(DriveTrainConstants.LEFTFRONTMOTOR_ID, MotorType.kBrushless);
  private final CANSparkMax leftBackMotor = new CANSparkMax(DriveTrainConstants.LEFTBACKMOTOR_ID, MotorType.kBrushless); 
  private final CANSparkMax rightFrontMotor = new CANSparkMax(DriveTrainConstants.RIGHTFRONTMOTOR_ID, MotorType.kBrushless); 
  private final CANSparkMax rightBackMotor = new CANSparkMax(DriveTrainConstants.RIGHTBACKMOTOR_ID, MotorType.kBrushless); 

  //Falcon motor controllers
  // private final WPI_TalonFX leftFrontMotor = new WPI_TalonFX(DriveTrainConstants.LEFTFRONTMOTOR_ID);
  // private final WPI_TalonFX leftBackMotor = new WPI_TalonFX(DriveTrainConstants.LEFTBACKMOTOR_ID);
  // private final WPI_TalonFX rightFrontMotor = new WPI_TalonFX(DriveTrainConstants.RIGHTFRONTMOTOR_ID);
  // private final WPI_TalonFX rightBackMotor = new WPI_TalonFX(DriveTrainConstants.RIGHTBACKMOTOR_ID);

  //Differentialdrive groups
  private final MotorControllerGroup m_leftDriveGroup = new MotorControllerGroup(leftFrontMotor, leftBackMotor);
  private final MotorControllerGroup m_rightDriveGroup = new MotorControllerGroup(rightFrontMotor, rightBackMotor);
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftDriveGroup, m_rightDriveGroup);

  //Odometry
  // private final DifferentialDriveOdometry m_odometry; 

  //Gyro
  private final ADIS16470_IMU gyro = new ADIS16470_IMU();

  //Compressor/Solenoids Inits
  private final Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);
  private final DoubleSolenoid shiftSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, DriveTrainConstants.SHIFTSOLFORWARD_ID, DriveTrainConstants.SHIFTSOLREVERSE_ID);

  //Booleans/Strings
  public String arcadeDriveSpeed = "default"; 


  /** Creates a new DriveTrain. */
  public DriveTrain(){
    //Factory Defaults

    //Motor follows
    leftBackMotor.follow(leftFrontMotor);
    rightBackMotor.follow(rightFrontMotor);
    m_leftDriveGroup.setInverted(true); //Invert one side drive motors

    coastDriveMotors(); //Start coasting drive motors
  
    // leftFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    // rightFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    // leftFrontMotor.setSelectedSensorPosition(0);
    // rightFrontMotor.setSelectedSensorPosition(0);
    
    //Odometry
    // m_odometry = new DifferentialDriveOdometry(
    //   getRotation2d(),
    //   encoderTicksToMeters(leftFrontMotor.getSelectedSensorPosition(), 4096, DriveTrainConstants.DRIVEGEARRATIO_LOW, DriveTrainConstants.DRIVEWHEELCIRCUMFERENCE),
    //   encoderTicksToMeters(rightFrontMotor.getSelectedSensorPosition(), 4096, DriveTrainConstants.DRIVEGEARRATIO_LOW, DriveTrainConstants.DRIVEWHEELCIRCUMFERENCE)
    // );

    // m_odometry.resetPosition(
    //   getRotation2d(), 
    //   encoderTicksToMeters(leftFrontMotor.getSelectedSensorPosition(), 4096, DriveTrainConstants.DRIVEGEARRATIO_LOW, DriveTrainConstants.DRIVEWHEELCIRCUMFERENCE), 
    //   encoderTicksToMeters(rightFrontMotor.getSelectedSensorPosition(), 4096, DriveTrainConstants.DRIVEGEARRATIO_LOW, DriveTrainConstants.DRIVEWHEELCIRCUMFERENCE), 
    //   new Pose2d()
    // );
  }

  public void arcadeDrive(double speed, double rotation){   //Our main ArcadeDrive command. 
    m_drive.arcadeDrive(speed, -rotation, false);
  } 
  public void arcadeDrive(double speed, double rotation, boolean isSquaredInputs){   //Secondary ArcadeDrive command. Has additional bool for squared inputs to increase controlability at low speeds. 
    m_drive.arcadeDrive(speed, -rotation, isSquaredInputs);
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
  
  public Rotation2d getRotation2d(){
    return Rotation2d.fromDegrees(gyro.getAngle());
  }
  // public Pose2d getPose2d(){
  //   return m_odometry.getPoseMeters();
  // }

  // public void resetEncoders(){
  //   leftFrontMotor.setSelectedSensorPosition(0);
  //   rightFrontMotor.setSelectedSensorPosition(0);
  // }
  // public void resetOdometry(Pose2d pose){
  //   resetEncoders();
  //   m_odometry.resetPosition(
  //     getRotation2d(), 
  //     encoderTicksToMeters(leftFrontMotor.getSelectedSensorPosition(), 4096, DriveTrainConstants.DRIVEGEARRATIO_LOW, DriveTrainConstants.DRIVEWHEELCIRCUMFERENCE), 
  //     encoderTicksToMeters(rightFrontMotor.getSelectedSensorPosition(), 4096, DriveTrainConstants.DRIVEGEARRATIO_LOW, DriveTrainConstants.DRIVEWHEELCIRCUMFERENCE), 
  //     new Pose2d());
  // }
  public void resetGyro(){
    gyro.reset();
  }

  public double encoderTicksToMeters(double currentEncoderValue, double encoderFullRev, double gearRatio, double wheelCircumferenceInInches){
    return ((currentEncoderValue / encoderFullRev) / gearRatio) * Units.inchesToMeters(wheelCircumferenceInInches);
  }


  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    compressor.enableDigital(); //Runs compressor
    getRotation2d();

    // m_odometry.update(
    //   getRotation2d(), 
    //   encoderTicksToMeters(leftFrontMotor.getSelectedSensorPosition(), 4096, DriveTrainConstants.DRIVEGEARRATIO_LOW, DriveTrainConstants.DRIVEWHEELCIRCUMFERENCE), 
    //   encoderTicksToMeters(rightFrontMotor.getSelectedSensorPosition(), 4096, DriveTrainConstants.DRIVEGEARRATIO_LOW, DriveTrainConstants.DRIVEWHEELCIRCUMFERENCE)
    // );
  }
}
