// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
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
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
  private final CANSparkMax leftArmMotor = new CANSparkMax(ArmConstants.LEFTARMMOTOR_ID, MotorType.kBrushless);
  private final CANSparkMax rightArmMotor = new CANSparkMax(ArmConstants.RIGHTARMMOTOR_ID, MotorType.kBrushless);
  private final Encoder armEncoder = new Encoder(ArmConstants.ARMENCODER_A, ArmConstants.ARMENCODER_B);

  private final DoubleSolenoid extenderSolenoid = new DoubleSolenoid(2, PneumaticsModuleType.REVPH, ArmConstants.EXTENDERSOLFORWARD_ID, ArmConstants.EXTENDERSOLREVERSE_ID);

  private final PIDController m_armPID = new PIDController(.1, 0, 0);

  public Arm() {
    rightArmMotor.follow(leftArmMotor);
    leftArmMotor.setIdleMode(IdleMode.kBrake);
    rightArmMotor.setIdleMode(IdleMode.kBrake); 

    ResetArmEncoder();

    m_armPID.setSetpoint(0);
  }

  public void MoveArm(double speed){
    leftArmMotor.set(speed);
  }
  public void StopArm(){
    leftArmMotor.set(0);
  }

  public double ArmEncoderRawValue(){
    return armEncoder.getRaw(); //8192 raw encoder ticks per rotation (full 360)
  }
  public double GetArmEncoderDegrees(){ 
    return ArmEncoderRawValue() / (8192/360); // X many tpr / 360 = ~22.755 ticks per degree 
  } //the math makes sense to me idk 
  public void ResetArmEncoder(){
    armEncoder.reset();
  }    

  public void SetArmToPoint(double desiredsetpoint, double currentpos){
    m_armPID.setSetpoint(desiredsetpoint);
    double PIDOutput = m_armPID.calculate(currentpos);

    leftArmMotor.set(PIDOutput);
  }


  public void ExtendArm(){
    extenderSolenoid.set(Value.kReverse);
  }
  public void RetractArm(){
    extenderSolenoid.set(Value.kForward);
  }
  public boolean GetArmExtended(){
    return (extenderSolenoid.get() == Value.kReverse);
  }
       


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm Degrees", GetArmEncoderDegrees());
  }
}
