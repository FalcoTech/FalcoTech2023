// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.Autos;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;

import java.util.ArrayList;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.RamseteAutoBuilder;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static DriveTrain m_drivetrain = new DriveTrain();
  public static Vision m_vision = new Vision();
  

  //Initialize driver station controllers
  public static final XboxController Pilot = new XboxController(OperatorConstants.PilotControllerPort);
  public static final XboxController CoPilot = new XboxController(OperatorConstants.CoPilotControllerPort);
  
  //Smartdashboard choosers/data
  SendableChooser<CommandBase> autoChooser = new SendableChooser<>(); //Autonomous chooser
  
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  //Things that should happen when the robot first initializes
  public RobotContainer() {
    configureBindings(); // Configure the trigger bindings
    configureSmartdashboard(); //Configures the smartdashboard settings/choosers
    PathPlannerServer.startServer(5811); //Start PathPlanner server to run with the app

    //Set Default Commands
    m_drivetrain.setDefaultCommand(new ArcadeDrive()); //Defaults the pilot's drive command
  }

  /** Use this method to define your trigger->command mappings. Triggers can be created via the {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@lin CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flightjoysticks}. */
  private void configureBindings() {
    //Pilot Controls
    new Trigger(() -> Pilot.getAButton()).onTrue(new InstantCommand(() -> m_drivetrain.shiftLowGear())); //Pilot's "A" button shifts to low gear
    new Trigger(() -> Pilot.getBButton()).onTrue(new InstantCommand(() -> m_drivetrain.shiftHighGear())); //Pilot's "B" button shifts to high gear

    new Trigger(() -> Pilot.getStartButton()).onTrue(new InstantCommand(() -> m_drivetrain.toggleArcadeDriveSpeed())); //Pilot's "Start" button toggles driver speed (charging pad)
    //Copilot Controls
  }

  private void configureSmartdashboard(){
    //Smartdashboard AutoChooser
    autoChooser.setDefaultOption("No Auto Selected", null);
    autoChooser.addOption("Left Side Cube Run", null);
    autoChooser.addOption("Right Side Cube Run", null);

    SmartDashboard.putData("Auto Mode", autoChooser);

  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected(); //Gets the autonomous mode selected on smartdashboard
  }
}
