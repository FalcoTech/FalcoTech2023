// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
  private final VictorSPX leftArmMotor = new VictorSPX(ArmConstants.LEFTARMMOTOR_ID);
  private final VictorSPX rightArmMotor = new VictorSPX(ArmConstants.RIGHTARMMOTOR_ID);

  // private final Encoder leftArmEncoder = new Encoder(1, 2);
  // private final Encoder rightArmEncoder = new Encoder(3, 4);

  // private final DoubleSolenoid extenderSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, tbd, tbd);

  /** Creates a new Arm. */
  public Arm() {
    rightArmMotor.follow(leftArmMotor);
    // rightArmMotor.setInverted(true);
    
  }

  public void MoveArm(double speed){
    leftArmMotor.set(ControlMode.PercentOutput, speed);
  }
  public void MoveArm(double leftSpeed, double rightSpeed){
    leftArmMotor.set(ControlMode.PercentOutput, leftSpeed);
    rightArmMotor.set(ControlMode.PercentOutput, rightSpeed);
    
  }

  public void ExtendArm(){
  //   if (armEncoder.getDistance() > 135 /*ENCODER NOT BETWEEN THIS VALUE AND THIS VALUE*/){
  //     extenderSolenoid.set(Value.kForward); //WILL PROBABLY NEED CHANGED
  //   }
  }
  public void RetractArm(){
  //   extenderSolenoid.set(Value.kReverse); //WILL PROBBABLY NEED CHANGED
  }

  // public double getLeftEncoderPosition(){
  //   return leftArmEncoder.getDistance();
  // }
  // public double getRightEncoderPosition(){
  //   return rightArmEncoder.getDistance();
  // }
  // public void resetArmEncoders(){
  //   leftArmEncoder.reset();
  //   rightArmEncoder.reset();
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
