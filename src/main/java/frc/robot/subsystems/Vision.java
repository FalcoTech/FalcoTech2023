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
  private final UsbCamera USBCamera;
  
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
    //USB CAM
    USBCamera = new UsbCamera(getName(), getName());
    CameraServer.startAutomaticCapture(); //start USB camera on RoboRIO
    cvSink = CameraServer.getVideo();

    //Apriltag detection
    photonCamera = new PhotonCamera(USBCamera.getName());
    detector = new AprilTagDetector();
    detector.addFamily(VisionConstants.tagFamily);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateVision(); //Update values of vision tracking
  }

  public void updateVision(){
    //LIMELIGHT
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    tv = table.getEntry("tv");
    ledMode = table.getEntry("ledMode");
    camMode = table.getEntry("camMode");

    // //PHOTON
    // photonResult = photonCamera.getLatestResult();
    // photonHasTarget = photonResult.hasTargets();
    // if (photonHasTarget){
    //   this.photonResult = photonResult;
    // }
  }

  public PhotonTrackedTarget getBestTarget(){
    if (photonHasTarget){
      return photonResult.getBestTarget();
    } else {
      return null;
    }
  }
  
}
