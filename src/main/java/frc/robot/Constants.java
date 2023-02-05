// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

public final class Constants {
  public static class OperatorConstants { 
    public static final int PILOTCONTROLLERPORT = 0;
    public static final int COPILOTCONTROLLERPORT = 1;
  }
  
  public static class DriveTrainConstants { 
    public static final int LEFTFRONTMOTOR_ID = 41;
    public static final int LEFTBACKMOTOR_ID = 43;
    public static final int RIGHTFRONTMOTOR_ID = 40;
    public static final int RIGHTBACKMOTOR_ID = 42;
    
    //DRIVE Encoder Values
    public static final int ENCODERFULLREV = 4096; //TBD
    public static final double GEARRATIO_LOW = 16.36;
    public static final double GEARRATIO_HIGH = 7.95; 

    public static final double WHEELRADIUSINCHES = 3; //6 inch wheels this year. /2 for radius
    public static final double WHEELCIRCUMFERENCEINCHES = 2 * Math.PI * WHEELRADIUSINCHES;
    public static final double TRACKWIDTH = Units.inchesToMeters(22); //TBD "Track width is the empirical measurement of the length from one center wheel to the opposite rail center wheel" - Google lol
    public static final DifferentialDriveKinematics DRIVEKINEMATICS = new DifferentialDriveKinematics(TRACKWIDTH); 


    //rpm DIVIDED BY gear ratio = free speed
    
    

    public static final int SHIFTSOLFORWARD_ID = 0;
    public static final int SHIFTSOLREVERSE_ID = 1;
  }

  public static class ArmConstants{
    public static final int LEFTARMMOTOR_ID = 23; //change when we find out
    public static final int RIGHTARMMOTOR_ID = 24; //CHANGE WHEN FIND OUT caps lol

    public static final int EXTENDERSOLFORWARD_ID = 2;
    public static final int EXTENDERSOLREVERSE_ID = 3;
  }

  public static class WristConstants{
    public static final int WRISTMOTOR_ID = 22;
  }

  public static class IntakeConstants{
    public static final int INTAKELEFTMOTOR_ID = 20;
    public static final int INTAKERIGHTMOTOR_ID = 21;
  }

  public static class VisionConstants{
    public static final String TAGFAMILY = "tag16h5";
  }

  public static class LEDsConstants{
    public static final int LEDSTRIPLEFTPORT = 8;
    public static final int LEDSTRIPRIGHTPORT = 9;
    public static final int LEDSTRIPLENGTH = 180; //maybe 90 each, maybe 180 each? idk lol
  }

  public static class AutoConstants {
    public static final HashMap<String, Command> AUTOEVENTMAP = new HashMap<>();
  }
  
}
