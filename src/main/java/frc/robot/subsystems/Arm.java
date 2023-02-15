// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
  // private final CANSparkMax armLeftMotor = new CANSparkMax(ArmConstants.LEFTARMMOTOR_ID, MotorType.kBrushless);
  // private final CANSparkMax armRightMotor = new CANSparkMax(ArmConstants.RIGHTARMMOTOR_ID, MotorType.kBrushless);
  // private final Encoder armEncoder = new Encoder(0, 1);


  // private final DoubleSolenoid extenderSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, ArmConstants.EXTENDERSOLFORWARD_ID, ArmConstants.EXTENDERSOLREVERSE_ID);


  /** Creates a new Arm. */
  public Arm() {
    // armRightMotor.follow(armLeftMotor, true);
    
    // armLeftMotor.setIdleMode(IdleMode.kBrake);
    // armRightMotor.setIdleMode(IdleMode.kBrake);
  }

  public void MoveArm(double speed){
    // armLeftMotor.set(speed);
  }

  public void ExtendArm(){
    // if (armEncoder.getDistance() > 135 /*ENCODER NOT BETWEEN THIS VALUE AND THIS VALUE*/){
    //   extenderSolenoid.set(Value.kForward); //WILL PROBABLY NEED CHANGED
    // }
  }
  
  public void RetractArm(){
    // extenderSolenoid.set(Value.kReverse); //WILL PROBBABLY NEED CHANGED
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
