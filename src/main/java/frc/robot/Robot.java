// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;


public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  
  //FMS Match Timer
  public static Timer matchTimer;


  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    //Initialize and stop match timer
    matchTimer = new Timer();
    matchTimer.reset();
    matchTimer.stop();
  }


   //This function is called every 20 ms, no matter the mode. Use this for items like diagnostics that you want ran during disabled, autonomous, teleoperated and test.
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled commands, running already-scheduled commands, removing finished or interrupted commands, and running subsystem periodic() methods.  This must be called from the robot's periodic block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    //Resets
    // SmartDashboard.putData("Reset Arm", new InstantCommand(() -> RobotContainer.m_arm.ResetArmEncoder()).ignoringDisable(true));
    // SmartDashboard.putData("Reset Wrist", new InstantCommand(() -> RobotContainer.m_wrist.ResetWristEncoder()).ignoringDisable(true));
    // SmartDashboard.putData("Reset Drive", new InstantCommand(() -> RobotContainer.m_drivetrain.ResetDriveEncoders()).ignoringDisable(true));
    // SmartDashboard.putData("Reset Gyro", new InstantCommand(() -> RobotContainer.m_drivetrain.ResetGyro()).ignoringDisable(true));
    // SmartDashboard.putData("RESET ALL", new InstantCommand(() -> RobotContainer.ResetAllSubsystems()).ignoringDisable(true));
  }

  @Override
  public void autonomousInit() {
    //Reset and start the match timer when match starts (Autonomous is initialized)
    matchTimer.reset();
    matchTimer.start();
    
    // schedule the autonomous command (example)
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    RobotContainer.m_drivetrain.ShiftLowGear();
  }

  @Override
  public void autonomousPeriodic() {}

  
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when teleop starts running. If you want the autonomous tocontinue until interrupted by another command, remove this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    RobotContainer.m_drivetrain.CoastDriveMotors();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

/**
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 */

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic(){
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
