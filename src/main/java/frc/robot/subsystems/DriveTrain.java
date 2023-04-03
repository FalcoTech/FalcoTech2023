// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveTrainConstants;


public class DriveTrain extends SubsystemBase {
  //Falcon motor controllers
  private final WPI_TalonFX leftFrontMotor = new WPI_TalonFX(DriveTrainConstants.LEFTFRONTMOTOR_ID);
  private final WPI_TalonFX leftBackMotor = new WPI_TalonFX(DriveTrainConstants.LEFTBACKMOTOR_ID);
  private final WPI_TalonFX rightFrontMotor = new WPI_TalonFX(DriveTrainConstants.RIGHTFRONTMOTOR_ID);
  private final WPI_TalonFX rightBackMotor = new WPI_TalonFX(DriveTrainConstants.RIGHTBACKMOTOR_ID);

  //Differentialdrive groups
  private final MotorControllerGroup m_leftDriveGroup = new MotorControllerGroup(leftFrontMotor, leftBackMotor);
  private final MotorControllerGroup m_rightDriveGroup = new MotorControllerGroup(rightFrontMotor, rightBackMotor);
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftDriveGroup, m_rightDriveGroup);

  //Odometry
  private final DifferentialDriveOdometry m_odometry; 
  private final ADIS16470_IMU gyro = new ADIS16470_IMU();
  public final Field2d m_field2d = new Field2d();

  //Compressor/Solenoids Inits
  private final Compressor compressor = new Compressor(2, PneumaticsModuleType.REVPH);
  private final DoubleSolenoid shiftSolenoid = new DoubleSolenoid(2, PneumaticsModuleType.REVPH, DriveTrainConstants.SHIFTSOLFORWARD_ID, DriveTrainConstants.SHIFTSOLREVERSE_ID);

  //Booleans/Strings
  public boolean m_slowDriveSpeed = false; 


  /** Creates a new DriveTrain. */
  public DriveTrain(){
    leftFrontMotor.configFactoryDefault();
    leftBackMotor.configFactoryDefault();
    rightFrontMotor.configFactoryDefault();
    rightBackMotor.configFactoryDefault();

    //Motor follows
    leftBackMotor.follow(leftFrontMotor);
    rightBackMotor.follow(rightFrontMotor);
    rightFrontMotor.setInverted(true);

    m_leftDriveGroup.setInverted(true); //Invert one side drive motors
    CoastDriveMotors(); //Start coasting drive motors
  
    leftFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    rightFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    ResetDriveEncoders();
    ResetGyro();

    m_odometry = new DifferentialDriveOdometry(
      GetRotation2d(), 
      EncoderTicksToMeters(leftFrontMotor.getSelectedSensorPosition()), 
      EncoderTicksToMeters(rightFrontMotor.getSelectedSensorPosition()));
     
    SmartDashboard.putData(m_field2d);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(
      GetRotation2d(), 
      EncoderTicksToMeters(leftFrontMotor.getSelectedSensorPosition()), 
      EncoderTicksToMeters(rightFrontMotor.getSelectedSensorPosition()));
    
    m_field2d.setRobotPose(GetPose2d());
    compressor.enableDigital();

    SmartDashboard.putNumber("Robot Pitch", GetGyroPitch());
    SmartDashboard.putNumber("Robot Angle", GetGyroYaw());

    SmartDashboard.putNumber("Left Drive Meters", GetLeftEncoderMeters());
    SmartDashboard.putNumber("Right Drive Meters", GetRightEncoderMeters());

  }


  public void ArcadeDrive(double speed, double rotation){   //Our main ArcadeDrive command. 
    m_drive.arcadeDrive(speed, rotation, false);
  } 
  public void ArcadeDrive(double speed, double rotation, boolean isSquaredInputs){   //Secondary ArcadeDrive command. Has additional bool for squared inputs to increase controlability at low speeds. 
    m_drive.arcadeDrive(speed, rotation, isSquaredInputs);
  } 
  public void TankDrive(double leftspeed, double rightspeed){
    m_drive.tankDrive(leftspeed, rightspeed);
  }
  public void TankDriveVolts(double leftVolts, double rightVolts){
    m_leftDriveGroup.setVoltage(leftVolts);
    m_rightDriveGroup.setVoltage(rightVolts);
    m_drive.feed();
  }


  public void ShiftLowGear(){
    shiftSolenoid.set(Value.kReverse);
  }
  public void ShiftHighGear(){
    shiftSolenoid.set(Value.kForward);
  }
  public boolean GetLowGear(){
    if (shiftSolenoid.get() == Value.kReverse){
      return true;
    } else{
      return false;
    }
  }

  public void BrakeDriveMotors(){
    leftFrontMotor.setNeutralMode(NeutralMode.Brake);
    leftBackMotor.setNeutralMode(NeutralMode.Brake);
    rightFrontMotor.setNeutralMode(NeutralMode.Brake);
    rightBackMotor.setNeutralMode(NeutralMode.Brake);
  }
  public void CoastDriveMotors(){
    leftFrontMotor.setNeutralMode(NeutralMode.Coast);
    leftBackMotor.setNeutralMode(NeutralMode.Coast);
    rightFrontMotor.setNeutralMode(NeutralMode.Coast);
    rightBackMotor.setNeutralMode(NeutralMode.Coast);
  }  
  public void ToggleArcadeDriveSpeed(){
    if (m_slowDriveSpeed){
      m_slowDriveSpeed = false;
      CoastDriveMotors();
    } else { 
      m_slowDriveSpeed = true;
      BrakeDriveMotors();
    }
  } 

  public Rotation2d GetRotation2d(){
    return Rotation2d.fromDegrees(gyro.getAngle());
  }

  public double GetGyroYaw(){
    return gyro.getAngle();
  }
  public double GetGyroPitch(){
    return gyro.getYComplementaryAngle();
  }
  public double GetGyroPitchAcceleration(){
    return gyro.getAccelY();
  }
  public double GetGyroRoll(){
    return gyro.getXComplementaryAngle();
  }

  public Pose2d GetPose2d(){
    return m_odometry.getPoseMeters();
  }

  public double EncoderTicksToMeters(double currentEncoderValue){
    double motorRotations = (double)currentEncoderValue / DriveTrainConstants.ENCODERFULLREV; 
    double wheelRotations = motorRotations / DriveTrainConstants.GEARRATIO_LOW; //Only in low gear
    double positionMeters = wheelRotations * DriveTrainConstants.WHEELCIRCUMFERENCEMETERS;
    return positionMeters;
  }

  public double GetLeftEncoderMeters(){
    return EncoderTicksToMeters(leftFrontMotor.getSelectedSensorPosition());
  }
  public double GetRightEncoderMeters(){
    return EncoderTicksToMeters(rightFrontMotor.getSelectedSensorPosition());
  }
  public double GetLeftEncoderVelocity(){
    return EncoderTicksToMeters(leftFrontMotor.getSelectedSensorVelocity()) * 10;
  }
  public double GetRightEncoderVelocity(){
    return EncoderTicksToMeters(rightFrontMotor.getSelectedSensorVelocity()) * 10;
  }
  public DifferentialDriveWheelSpeeds GetWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds(GetLeftEncoderVelocity(), GetRightEncoderVelocity());
  }


  public void ResetDriveEncoders(){
    leftFrontMotor.setSelectedSensorPosition(0);
    leftBackMotor.setSelectedSensorPosition(0);
    rightFrontMotor.setSelectedSensorPosition(0);
    rightBackMotor.setSelectedSensorPosition(0);
  }
  public void ResetGyro(){
    gyro.reset();
  }
  public void ResetOdometry(Pose2d pose){
    ResetDriveEncoders();
    m_odometry.resetPosition(
      GetRotation2d(), 
      EncoderTicksToMeters(leftFrontMotor.getSelectedSensorPosition()), 
      EncoderTicksToMeters(rightFrontMotor.getSelectedSensorPosition()), 
      pose);
  }
}
