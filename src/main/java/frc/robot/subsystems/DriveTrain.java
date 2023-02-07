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
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveTrainConstants;


public class DriveTrain extends SubsystemBase {
  //Sparkmax motor controllers (old)
  // private final CANSparkMax leftFrontMotor = new CANSparkMax(DriveTrainConstants.LEFTFRONTMOTOR_ID, MotorType.kBrushless);
  // private final CANSparkMax leftBackMotor = new CANSparkMax(DriveTrainConstants.LEFTBACKMOTOR_ID, MotorType.kBrushless); 
  // private final CANSparkMax rightFrontMotor = new CANSparkMax(DriveTrainConstants.RIGHTFRONTMOTOR_ID, MotorType.kBrushless); 
  // private final CANSparkMax rightBackMotor = new CANSparkMax(DriveTrainConstants.RIGHTBACKMOTOR_ID, MotorType.kBrushless); 

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

  //Gyro
  private final ADIS16470_IMU gyro = new ADIS16470_IMU();

  //Field Image
  private final Field2d m_field2d = new Field2d();

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

    CoastDriveMotors(); //Start coasting drive motors
  
    leftFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    rightFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    
    resetEncoders();
    resetGyro();


    m_odometry = new DifferentialDriveOdometry(
    GetRotation2d(), 
    encoderTicksToMeters(leftFrontMotor.getSelectedSensorPosition()), 
    encoderTicksToMeters(rightFrontMotor.getSelectedSensorPosition()));
  }

  public void arcadeDrive(double speed, double rotation){   //Our main ArcadeDrive command. 
    m_drive.arcadeDrive(speed, -rotation, false);
  } 
  public void arcadeDrive(double speed, double rotation, boolean isSquaredInputs){   //Secondary ArcadeDrive command. Has additional bool for squared inputs to increase controlability at low speeds. 
    m_drive.arcadeDrive(speed, -rotation, isSquaredInputs);
  } 

  public void ShiftLowGear(){
    shiftSolenoid.set(Value.kForward);
  }
  public void ShiftHighGear(){
    shiftSolenoid.set(Value.kReverse);
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
    if (arcadeDriveSpeed == "default"){
      arcadeDriveSpeed = "slow";
      BrakeDriveMotors();
    } else{
      arcadeDriveSpeed = "default";
      CoastDriveMotors();
    }
  } 
  
  public Rotation2d GetRotation2d(){
    return Rotation2d.fromDegrees(gyro.getAngle());
  }
  public Pose2d GetPose2d(){
    return m_odometry.getPoseMeters();
  }
  
  public double GetLeftEncoderVelocity(){
    return -encoderTicksToMeters(leftFrontMotor.getSelectedSensorVelocity());
  } //Check to see if these values return the same when driving straight
  public double GetRightEncoderVelocity(){
    return -encoderTicksToMeters(rightFrontMotor.getSelectedSensorPosition()) * 10;
  }


  public double encoderTicksToMeters(double currentEncoderValue){
    double motorRotations = (double)currentEncoderValue / DriveTrainConstants.ENCODERFULLREV; //FULLREV may need to be 2048 idk
    double wheelRotations = motorRotations / DriveTrainConstants.GEARRATIO_LOW;
    double positionMeters = wheelRotations * DriveTrainConstants.WHEELCIRCUMFERENCEMETERS;
    return positionMeters;
  }

  public void resetEncoders(){
    leftFrontMotor.setSelectedSensorPosition(0);
    rightFrontMotor.setSelectedSensorPosition(0);
    leftBackMotor.setSelectedSensorPosition(0);
    rightBackMotor.setSelectedSensorPosition(0);
  }
  public void resetGyro(){
    gyro.reset();
  }
  public void resetOdometry(Pose2d pose){
    resetEncoders();
    m_odometry.resetPosition(
      GetRotation2d(), 
      encoderTicksToMeters(leftFrontMotor.getSelectedSensorPosition()), 
      encoderTicksToMeters(rightFrontMotor.getSelectedSensorPosition()), 
      new Pose2d());
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    compressor.enableDigital(); //Runs compressor
    GetRotation2d();

    m_odometry.update(
      GetRotation2d(), 
      encoderTicksToMeters(leftFrontMotor.getSelectedSensorPosition()), 
      encoderTicksToMeters(rightFrontMotor.getSelectedSensorPosition()));
    
      m_field2d.setRobotPose(GetPose2d());
    
  }
}
