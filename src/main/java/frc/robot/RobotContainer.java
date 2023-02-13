// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//Import all (*) constants, subsystems and commands
import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.commands.LEDs.RunRainbow;
import frc.robot.commands.LEDs.SolidPurple;
import frc.robot.subsystems.*;

import com.pathplanner.lib.auto.RamseteAutoBuilder;
import com.pathplanner.lib.commands.PPRamseteCommand;
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;


public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static DriveTrain m_drivetrain = new DriveTrain();
  public static Arm m_arm = new Arm();
  public static Wrist m_wrist = new Wrist();
  public static Intake m_intake = new Intake();
  public static Vision m_vision = new Vision();
  public static LEDs m_leds = new LEDs();

  public static final XboxController Pilot = new XboxController(OperatorConstants.PILOTCONTROLLERPORT);
  public static final XboxController CoPilot = new XboxController(OperatorConstants.COPILOTCONTROLLERPORT);

  //Smartdashboard choosers/data
  SendableChooser<CommandBase> m_autoChooser = new SendableChooser<>();
  
  //Pathplanner

  RamseteAutoBuilder autoBuilder = new RamseteAutoBuilder(
    m_drivetrain::GetPose2d, 
    m_drivetrain::ResetOdometry, 
    new RamseteController(), 
    DriveTrainConstants.DRIVEKINEMATICS, 
    null, 
    AutoConstants.AUTOEVENTMAP, 
    m_drivetrain);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */ //Things that should happen when the robot first initializes
  public RobotContainer() {
    configureBindings(); 
    configureSmartdashboard(); 
    PathPlannerServer.startServer(5811); 

    //Set Default Commands
    m_drivetrain.setDefaultCommand(new ArcadeDrive()); 
    m_arm.setDefaultCommand(new RunArm());
    m_intake.setDefaultCommand(new RunIntake());
  }

  /** Use this method to define your trigger->command mappings. Triggers can be created via the {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@lin CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flightjoysticks}. */
  private void configureBindings() {
    //Pilot Controls
    new Trigger(() -> Pilot.getAButton()).onTrue(new InstantCommand(() -> m_drivetrain.ShiftLowGear())); //Pilot's "A" button shifts to low gear
    new Trigger(() -> Pilot.getBButton()).onTrue(new InstantCommand(() -> m_drivetrain.ShiftHighGear())); //Pilot's "B" button shifts to high gear
    new Trigger(() -> Pilot.getStartButton()).onTrue(new InstantCommand(() -> m_drivetrain.ToggleArcadeDriveSpeed())); //Pilot's "Start" button toggles driver speed (charging pad)
  
    
    //Copilot Controls
    new Trigger(() -> CoPilot.getStartButton()).whileFalse(new InstantCommand(() -> m_leds.setDefaultCommand(new RunRainbow()))); //no shot this works 💀
    new Trigger(() -> CoPilot.getStartButton()).whileTrue(new InstantCommand(() -> m_leds.setDefaultCommand(new SolidPurple()))); //no shot this works 💀

    //Arm extender
    new Trigger(() -> CoPilot.getLeftBumper()).onTrue(new InstantCommand(() -> m_arm.ExtendArm()));
    new Trigger(() -> CoPilot.getRightBumper()).onTrue(new InstantCommand(() -> m_arm.RetractArm())); 
    
    //LEDS IDEA: HAVE THE DEFAULT COMMAND SWITCH WITH COPILOT INPUT.  
  }

  private void configureSmartdashboard(){
    //Smartdashboard AutoChooser options
    m_autoChooser.setDefaultOption("No Auto Selected", null);
    m_autoChooser.addOption("Left Side Cube Run", null);
    m_autoChooser.addOption("Right Side Cube Run", null);
    SmartDashboard.putData("Auto Mode", m_autoChooser); // Add chooser for auto


  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected(); //Gets the autonomous mode selected on smartdashboard
  }

  
}
