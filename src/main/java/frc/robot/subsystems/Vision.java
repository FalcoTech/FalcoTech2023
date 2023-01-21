// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.ObjectInputStream.GetField;
import java.util.Map;
import java.util.Optional;
import java.util.TreeMap;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


public class Vision extends SubsystemBase {
  //USB Camera(s)
  // private final UsbCamera USBCamera;
  //AprilTag stuff
  private final PhotonCamera photonCamera;
  private static boolean photonHasTarget;
  private static PhotonPipelineResult photonResult;
  private final CvSink cvSink;
  private final AprilTagDetector detector;

  //Limelight values
  public static NetworkTable table;
  public static NetworkTableEntry tx;
  public static NetworkTableEntry ty;
  public static NetworkTableEntry ta;
  public static NetworkTableEntry tv;
  public static NetworkTableEntry camMode;
  public static NetworkTableEntry ledMode;

  public Vision() {
    //Start USB camera on RoboRIO
    CameraServer.startAutomaticCapture(); //start USB camera on RoboRIO
    cvSink = CameraServer.getVideo();

    //Apriltag stuff
    // photonCamera = new PhotonCamera("Microsoft® LifeCam HD-3000");
    photonCamera = new PhotonCamera("photonvision");
    detector = new AprilTagDetector();
    detector.addFamily(VisionConstants.tagFamily);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
