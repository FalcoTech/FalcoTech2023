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



public class Vision extends SubsystemBase {
  //USB Camera(s)

  private final CvSink m_cvSink;
  
  //Limelight
  private NetworkTable table;
  private NetworkTableEntry tValidTarget;
  private NetworkTableEntry tOffsetX;
  private NetworkTableEntry tOffsetY;
  private NetworkTableEntry tArea;

  private NetworkTableEntry tShortestLength;
  private NetworkTableEntry tLongestLength;
  private NetworkTableEntry tHorizontalLength;
  private NetworkTableEntry tVerticalLength;


  public Vision() {
    CameraServer.startAutomaticCapture(); //start USB camera on RoboRIO
    m_cvSink = CameraServer.getVideo();

    table = NetworkTableInstance.getDefault().getTable("limelight"); //Limelight table
    //Limelight entries
    tValidTarget = table.getEntry("tv"); 
    tOffsetX = table.getEntry("tx");
    tOffsetY = table.getEntry("ty");
    tArea = table.getEntry("ta");
  }

  public void updateLimelight(){
    double tv = tValidTarget.getDouble(0.0);
    double tx = tOffsetX.getDouble(0.0);
    double ty = tOffsetY.getDouble(0.0);
    double ta = tArea.getDouble(0.0);
  }


  @Override
  public void periodic() { // This method will be called once per scheduler run
    updateLimelight();
  }
}
