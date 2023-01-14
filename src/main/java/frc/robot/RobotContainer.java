// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.Autos;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static DriveTrain m_drivetrain = new DriveTrain();


  public static final XboxController Pilot = new XboxController(OperatorConstants.PilotControllerPort);
  public static final XboxController CoPilot = new XboxController(OperatorConstants.CoPilotControllerPort);
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    m_drivetrain.setDefaultCommand(new ArcadeDrive());
    // SmartDashboard.putData();
  }

  /** Use this method to define your trigger->command mappings. Triggers can be created via the {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@lin CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flightjoysticks}. */
  private void configureBindings() {
    //Pilot Controls
    new Trigger(() -> Pilot.getAButton()).onTrue(new InstantCommand(() -> m_drivetrain.shiftLowGear())); //Pilot's "A" button shifts to low gear
    new Trigger(() -> Pilot.getBButton()).onTrue(new InstantCommand(() -> m_drivetrain.shiftHighGear())); //Pilot's "B" button shifts to high gear
    new Trigger(() -> Pilot.getYButton()).onTrue(new InstantCommand(() -> m_drivetrain.toggleGear())); //Pilot's "Y" button toggles gears

    new Trigger(() -> Pilot.getStartButton()).onTrue(new InstantCommand(() -> m_drivetrain.toggleArcadeDriveSpeed())); //Pilot's "Start" button toggles driver speed (charging pad)
  }


/**
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }*/
}
