// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//Import all (*) constants, subsystems and commands
import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.commands.LEDs.*;
import frc.robot.commands.armpresets.HumanPlayerPosArm;
import frc.robot.commands.armpresets.ScoringPosArm;
import frc.robot.commands.armpresets.ZeroArm;
import frc.robot.subsystems.*;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.RamseteAutoBuilder;
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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
  public static Intake m_intake = new Intake();
  public static Vision m_vision = new Vision();
  public static LEDs m_leds = new LEDs();

  public static final XboxController Pilot = new XboxController(OperatorConstants.PILOTCONTROLLERPORT);
  public static final XboxController CoPilot = new XboxController(OperatorConstants.COPILOTCONTROLLERPORT);

  //Smartdashboard choosers/data
  SendableChooser<Command> m_autoChooser = new SendableChooser<>();
  
  //Pathplanner
  public static HashMap<Command, String> autoMap = new HashMap<>();
  public Command ramAutoBuilder(String pathName, HashMap<String, Command> eventMap){
    RamseteAutoBuilder pathBuilder = new RamseteAutoBuilder(
      m_drivetrain::GetPose2d, 
      m_drivetrain::ResetOdometry, 
      new RamseteController(2, .7),//TBD
      DriveTrainConstants.DRIVEKINEMATICS, 
      new SimpleMotorFeedforward(
        0.032, //these 
        .48, //change
        .07 //speed
      ), 
      m_drivetrain::GetWheelSpeeds, 
      new PIDConstants(.033, 0, 0), //this prob doesn't idk
      m_drivetrain::TankDriveVolts,
      eventMap,
      true,
      m_drivetrain
    );
    List<PathPlannerTrajectory> pathToFollow = PathPlanner.loadPathGroup(pathName, PathPlanner.getConstraintsFromPath(pathName));
    final Command auto = pathBuilder.fullAuto(pathToFollow);
    autoMap.put(auto, pathName);
    return auto;
  }
  


  /** The container for the robot. Contains subsystems, OI devices, and commands. */ //Things that should happen when the robot first initializes
  public RobotContainer() {
    configureBindings(); 
    configureSmartdashboard(); 

    PathPlannerServer.startServer(5811); 
    autoMap.put(new InstantCommand(), "Nothing");

    //Set Default Commands
    m_drivetrain.setDefaultCommand(new ArcadeDrive()); 
    m_arm.setDefaultCommand(new RunArm());
    m_intake.setDefaultCommand(new RunIntake());
    // m_leds.setDefaultCommand(new RunRainbow());
  }

  /** Use this method to define your trigger->command mappings. Triggers can be created via the {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@lin CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flightjoysticks}. */
  private void configureBindings() {
    //Pilot Controls
    new Trigger(() -> Pilot.getAButton()).onTrue(new InstantCommand(() -> m_drivetrain.ShiftLowGear())); //Pilot's "A" button shifts to low gear
    new Trigger(() -> Pilot.getBButton()).onTrue(new InstantCommand(() -> m_drivetrain.ShiftHighGear())); //Pilot's "B" button shifts to high gear
    new Trigger(() -> Pilot.getStartButton()).onTrue(new InstantCommand(() -> m_drivetrain.ToggleArcadeDriveSpeed())); //Pilot's "Start" button toggles driver speed (charging pad)
  
    //Copilot Controls
    new Trigger(() -> CoPilot.getLeftBumper()).onTrue(new InstantCommand(() -> m_arm.ExtendArm()));
    new Trigger(() -> CoPilot.getRightBumper()).onTrue(new InstantCommand(() -> m_arm.RetractArm())); 

    new Trigger(() -> CoPilot.getAButton()).onTrue(new InstantCommand(() -> m_arm.setDefaultCommand(new ZeroArm())));
    new Trigger(() -> CoPilot.getBButton()).onTrue(new InstantCommand(() -> m_arm.setDefaultCommand(new ScoringPosArm())));
    new Trigger(() -> CoPilot.getXButton()).onTrue(new InstantCommand(() -> m_arm.setDefaultCommand(new HumanPlayerPosArm())));

  }

  private void configureSmartdashboard(){
    //Smartdashboard AutoChooser options
    m_autoChooser.setDefaultOption("No Auto Selected", new InstantCommand());
    m_autoChooser.addOption("Left Side Cube Run", null);
    m_autoChooser.addOption("Right Side Cube Run", null);
    m_autoChooser.addOption("First Test Path", ramAutoBuilder("First Test", AutoConstants.AUTOEVENTMAP));
    SmartDashboard.putData("Auto Mode", m_autoChooser); // Add chooser for auto

  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected(); //Gets the autonomous mode selected on smartdashboard
  }

  
}
