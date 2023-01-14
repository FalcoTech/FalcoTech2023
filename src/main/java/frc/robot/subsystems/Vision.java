// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;
import java.util.TreeMap;

import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Vision extends SubsystemBase {
  //USB Camera(s)
  private final UsbCamera camera;
  //AprilTag stuff
  private final CvSink cvSink;
  private final AprilTagDetector tagDetector;
  //Limelight values
  public static NetworkTable table;
  public static NetworkTableEntry tx;
  public static NetworkTableEntry ty;
  public static NetworkTableEntry ta;
  public static NetworkTableEntry tv;
  public static NetworkTableEntry camMode;
  public static NetworkTableEntry ledMode;
  
  public Vision() {
    camera = CameraServer.startAutomaticCapture();
    cvSink = CameraServer.getVideo(); 

    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    tv = table.getEntry("tv");
    ledMode = table.getEntry("ledMode");
    camMode = table.getEntry("camMode");

    tagDetector = new AprilTagDetector();
    tagDetector.addFamily(Constants.VisionConstants.tagFamily);
  }

  public double getTargetOffsetX(){
    return tx.getDouble(0.0);
  }
  public double getTargetOffsetY(){
    return ty.getDouble(0.0);
  }
  public double getTargetArea(){
    return ta.getDouble(0.0);
  }
  public boolean getValidTarget(){
    return (tv.getDouble(0.0) == 1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


}
