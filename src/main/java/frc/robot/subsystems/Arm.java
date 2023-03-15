// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.fasterxml.jackson.databind.jsontype.PolymorphicTypeValidator.Validity;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
  private final VictorSPX leftArmMotor = new VictorSPX(ArmConstants.LEFTARMMOTOR_ID);
  private final VictorSPX rightArmMotor = new VictorSPX(ArmConstants.RIGHTARMMOTOR_ID);
  private final Encoder armEncoder = new Encoder(ArmConstants.ARMENCODER_A, ArmConstants.ARMENCODER_B);

  private final PIDController m_armPID = new PIDController(.5, 0, 0);

  private final DoubleSolenoid extenderSolenoid = new DoubleSolenoid(2, PneumaticsModuleType.REVPH, ArmConstants.EXTENDERSOLFORWARD_ID, ArmConstants.EXTENDERSOLREVERSE_ID);


  public Arm() {
    rightArmMotor.follow(leftArmMotor);
    //START EXTENDER NOT EXTENDED
    ResetArmEncoder();
  }

  public void MoveArm(double speed){
    leftArmMotor.set(ControlMode.PercentOutput, speed);
  }
  public void StopArm(){
    leftArmMotor.set(ControlMode.PercentOutput, 0);
  }
  public void SetArmToPoint(double currentpos, double setpoint){
    leftArmMotor.set(ControlMode.PercentOutput, m_armPID.calculate(currentpos, setpoint));
  }

  public double GetArmEncoderPosition(){
    return armEncoder.getDistance();
  }
  public void ResetArmEncoder(){
    armEncoder.reset();
  }

  public double GetArmMotorOutputPercent(){
    return leftArmMotor.getMotorOutputPercent();
  }
  public double GetArmMotorOutputVolts(){
    return leftArmMotor.getMotorOutputVoltage();
  }

  public void ExtendArm(){
    // if (GetArmEncoderPosition() < -500 && GetArmEncoderPosition() > 200){
    //   extenderSolenoid.set(Value.kReverse);
    // }
    extenderSolenoid.set(Value.kReverse);
  }
  public void RetractArm(){
    extenderSolenoid.set(Value.kForward);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // if (GetArmEncoderPosition() > -500 && GetArmEncoderPosition() < 200){
    //   RetractArm();
    // }
    SmartDashboard.putNumber("Arm Encoder Value:", GetArmEncoderPosition());
    SmartDashboard.putNumber("Arm Motor Output Percent", GetArmMotorOutputPercent());
    SmartDashboard.putNumber("Arm Motor Output Volts", GetArmMotorOutputVolts());
    
  }
}
