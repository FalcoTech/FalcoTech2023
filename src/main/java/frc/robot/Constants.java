// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants { //Constants for OI/Driver controls 
    //Controller Ports
    public static final int PILOTCONTROLLERPORT = 0;
    public static final int COPILOTCONTROLLERPORT = 1;
  }
  
  public static class DriveTrainConstants { //Constants for drive train stuff like motors/pneumatics
    //Motor ID's
    public static final int LEFTFRONTMOTOR_ID = 41;
    public static final int LEFTBACKMOTOR_ID = 43;
    public static final int RIGHTFRONTMOTOR_ID = 40;
    public static final int RIGHTBACKMOTOR_ID = 42;
    //Encoder Values
    public static final int DRIVETICKSPERREVOLUTION = 42;
    public static final double DRIVEGEARRATIO_LOW = 16.36;
    public static final double DRIVEGEARRATIO_HIGH = 7.95; 
    //rpm DIVIDED BY gear ratio = free speed

    public static final double DRIVEDISTPERREVOLUTION = 2 * Math.PI * .0762; //in meters
    //6 INCH WHEELS THIS YEAR
    public static final double DRIVEWHEELCIRCUMFERENCE = 2 * Math.PI * (4/2); /*radius in*/
    
    //Shifter solenoid ID's
    public static final int SHIFTSOLFORWARD_ID = 0;
    public static final int SHIFTSOLREVERSE_ID = 1;
  }
  //Arm constants
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

  public static class PathPlannerConstants {
    public static final HashMap<String, Command> AUTOEVENTMAP = new HashMap<>();
  }
  
}
