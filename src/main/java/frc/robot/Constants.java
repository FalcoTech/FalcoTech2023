// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
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
    public static final int PilotControllerPort = 0;
    public static final int CoPilotControllerPort = 1;
  }
  
  public static class DriveTrainConstants { //Constants for drive train stuff like motors/pneumatics
    public static final int leftFrontMotor_ID = 41;
    public static final int leftBackMotor_ID = 43;
    public static final int rightFrontMotor_ID = 40;
    public static final int rightBackMotor_ID = 42;
    public static final int shiftSolForward_ID = 0;
    public static final int shiftSolReverse_ID = 1;
  }

  public static class VisionConstants {
    public static final String tagFamily = "tag16h5";
  }

  
}
