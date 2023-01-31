// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class RunIntake extends CommandBase {
  /** Creates a new RunIntake. */
  public RunIntake() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double CoPilotLeftTrigger = RobotContainer.CoPilot.getLeftTriggerAxis();
    double CoPilotRightTrigger = RobotContainer.CoPilot.getRightTriggerAxis();

    //This obviously won't work off the bat, but it's a good proof of concept
    if (RobotContainer.m_intake.intakeColorSensor.getColor() != new Color(255, 0, 255) 
    || RobotContainer.m_intake.intakeColorSensor.getColor() != new Color(255, 255, 0)){
      RobotContainer.m_intake.runIntake(CoPilotRightTrigger * .9);
    } else {
      RobotContainer.m_intake.runIntake(-CoPilotLeftTrigger * .9);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
