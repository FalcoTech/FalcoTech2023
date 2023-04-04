// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//Import all (*) constants, subsystems and commands
import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.commands.armpresets.*;
import frc.robot.commands.autos.*;
import frc.robot.commands.wristpresets.*;
import frc.robot.subsystems.*;
import pabeles.concurrency.ConcurrencyOps.Reset;

import java.util.HashMap;
import java.util.List;

import javax.print.attribute.standard.Copies;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.RamseteAutoBuilder;
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;


public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static DriveTrain m_drivetrain = new DriveTrain();
  public static Arm m_arm = new Arm();
  public static Wrist m_wrist = new Wrist();
  public static Intake m_intake = new Intake();
  public static Vision m_vision = new Vision();
  public static LEDs m_leds = new LEDs();

  public static final XboxController Pilot = new XboxController(OperatorConstants.PILOTCONTROLLERPORT);
  // public static final PS4Controller Pilot = new PS4Controller(OperatorConstants.PILOTCONTROLLERPORT);
  public static final XboxController CoPilot = new XboxController(OperatorConstants.COPILOTCONTROLLERPORT);

  //Smartdashboard choosers/data
  SendableChooser<Command> m_autoChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */ //Things that should happen when the robot first initializes
  public RobotContainer() {
    configureBindings(); 
    configureSmartdashboard(); 


    //Set Default Commands
    m_drivetrain.setDefaultCommand(new ArcadeDrive()); 
    m_arm.setDefaultCommand(new RunArm());
    m_intake.setDefaultCommand(new RunIntake());
    m_wrist.setDefaultCommand(new RunWrist());
  }

  /** Use this method to define your trigger->command mappings. Triggers can be created via the {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@lin CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flightjoysticks}. */
  private void configureBindings() {
    //Pilot Controls
    // new Trigger(() -> Pilot.getCrossButton()).onTrue(new InstantCommand(() -> m_drivetrain.ShiftLowGear())); //Pilot's "A" button shifts to low gear
    // new Trigger(() -> Pilot.getCircleButton()).onTrue(new InstantCommand(() -> m_drivetrain.ShiftHighGear())); //Pilot's "B" button shifts to high gear
    new Trigger(() -> Pilot.getAButton()).onTrue(new InstantCommand(() -> m_drivetrain.ShiftLowGear())); //Pilot's "A" button shifts to low gear
    new Trigger(() -> Pilot.getBButton()).onTrue(new InstantCommand(() -> m_drivetrain.ShiftHighGear())); //Pilot's "B" button shifts to high gear
    
    new Trigger(() -> Pilot.getLeftBumper()).onTrue(new InstantCommand(() -> m_drivetrain.SlowArcadeDriveSpeed())); //Pilot's "B" button shifts to high gear
    new Trigger(() -> Pilot.getRightBumper()).onTrue(new InstantCommand(() -> m_drivetrain.NormalArcadeDriveSpeed())); //Pilot's "B" button shifts to high gear
    new Trigger(() -> Pilot.getStartButton()).onTrue(new InstantCommand(() -> m_drivetrain.ToggleArcadeDriveSpeed())); //Pilot's "Start" button toggles driver speed (charging pad)
    // new Trigger(() -> Pilot.getOptionsButton()).onTrue(new InstantCommand(() -> m_drivetrain.ToggleArcadeDriveSpeed())); //Pilot's "Start" button toggles driver speed (charging pad)
    // new Trigger(() -> Pilot.getBackButton()).onTrue(new InstantCommand(() -> m_drivetrain.ResetEncoders())); //Pilot's "Start" button toggles driver speed (charging pad)

    //Copilot Controls
    new Trigger(() -> CoPilot.getStartButton()).onTrue(new InstantCommand(() -> m_leds.SwitchHPColor()));

    new Trigger(() -> CoPilot.getLeftBumper()).onTrue(new InstantCommand(() -> m_arm.ExtendArm()));
    new Trigger(() -> CoPilot.getRightBumper()).onTrue(new InstantCommand(() -> m_arm.RetractArm())); 

    new Trigger(() -> CoPilot.getXButton()).onTrue(new InstantCommand(() -> m_wrist.setDefaultCommand(new ZeroWrist())));
    new Trigger(() -> CoPilot.getAButton()).onTrue(new InstantCommand(() -> m_wrist.setDefaultCommand(new HalfTurnWrist())));
    new Trigger(() -> CoPilot.getBButton()).onTrue(new InstantCommand(() -> m_wrist.setDefaultCommand(new FullTurnWrist())));

    new Trigger(() -> CoPilot.getPOV() == 0).onTrue(new InstantCommand(() -> m_arm.setDefaultCommand(new HighNodeArm())));
    new Trigger(() -> CoPilot.getPOV() == 90).onTrue(new InstantCommand(() -> m_arm.setDefaultCommand(new MidNodeArm())));
    new Trigger(() -> CoPilot.getPOV() == 180).onTrue(new InstantCommand(() -> m_arm.setDefaultCommand(new ZeroArm())));
    new Trigger(() -> CoPilot.getPOV() == 270).onTrue(new InstantCommand(() -> m_arm.setDefaultCommand(new HumanPlayerArm())));
    new Trigger(() -> CoPilot.getBackButton()).onTrue(new InstantCommand(() -> m_arm.setDefaultCommand(new GroundPickupArm())));
  }

  private void configureSmartdashboard(){
    //Smartdashboard AutoChooser options
    m_autoChooser.setDefaultOption("No Auto Selected", new InstantCommand());
    m_autoChooser.addOption("Place Cone & Drive Out", new PlaceConeMidDriveOutFullAuto());
    m_autoChooser.addOption("Balance", new BalanceFullAuto());

    SmartDashboard.putData("Auto Mode", m_autoChooser); // Add chooser for auto
    
    //Resets
    SmartDashboard.putData("Reset Arm", new InstantCommand(() -> m_arm.ResetArmEncoder()).ignoringDisable(true));
    SmartDashboard.putData("Reset Wrist", new InstantCommand(() -> m_wrist.ResetWristEncoder()).ignoringDisable(true));
    SmartDashboard.putData("Reset Drive", new InstantCommand(() -> m_drivetrain.ResetDriveEncoders()).ignoringDisable(true));
    SmartDashboard.putData("Reset Gyro", new InstantCommand(() -> m_drivetrain.ResetGyro()).ignoringDisable(true));
    SmartDashboard.putData("RESET ALL", new InstantCommand(() -> ResetAllSubsystems()).ignoringDisable(true));
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected(); //Gets the autonomous mode selected on smartdashboard
  }

  public static void ResetAllSubsystems(){
    m_arm.ResetArmEncoder();
    m_wrist.ResetWristEncoder();
    m_drivetrain.ResetDriveEncoders();
    m_drivetrain.ResetGyro();
  }
}
