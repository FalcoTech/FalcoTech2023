// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.lib.LimelightHelpers;



public class Vision extends SubsystemBase {
  //USB Camera(s)
  private final CvSink m_cvSink;
  
  

  public Vision() {
    CameraServer.startAutomaticCapture(); //start USB camera on RoboRIO
    m_cvSink = CameraServer.getVideo();

  }

  public void LLLEDsOFF(){
    LimelightHelpers.setLEDMode_ForceOff("");
  }
  public void LimelightLEDsOn(){
    LimelightHelpers.setLEDMode_ForceOn("");
  }


  @Override
  public void periodic() { // This method will be called once per scheduler run

  }
}
