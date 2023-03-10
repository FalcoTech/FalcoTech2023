// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.WristConsatnts;

public class Wrist extends SubsystemBase {
  private final VictorSPX wristMotor = new VictorSPX(WristConsatnts.WRISTMOTOR_ID);
  private final Encoder wristEncoder = new Encoder(WristConsatnts.WRISTENCODER_A, WristConsatnts.WRISTENCODER_B);


  /** Creates a new Wrist. */
  public Wrist() {
    ResetWristEncoder();
  }

  public void TurnWrist(double speed){
    wristMotor.set(ControlMode.PercentOutput, speed);
  }
  
  public double GetWristEncoderPosition(){
    return wristEncoder.getDistance();
  }
  public void ResetWristEncoder(){
    wristEncoder.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Wrist Encoder Value:", GetWristEncoderPosition());
    SmartDashboard.putNumber("Wrist Motor Output Percent", wristMotor.getMotorOutputPercent()); //I think this does the output of the motor controller itself, not the actual motor. 
    SmartDashboard.putNumber("Wrist Motor Output Voltage", wristMotor.getMotorOutputVoltage()); //applied voltage to motor in volts
  }
}
