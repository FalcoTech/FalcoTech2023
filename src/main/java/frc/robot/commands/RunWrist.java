// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class RunWrist extends CommandBase {
  /** Creates a new RunWrist. */
  public RunWrist() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double CoPilotLeftX = RobotContainer.CoPilot.getLeftX();
  
    RobotContainer.m_wrist.TurnWrist(-CoPilotLeftX); //reverse sign for the other way
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.CoPilot.getAButton() || RobotContainer.CoPilot.getBButton(); //ANY BUTTON THAT YOU WANT A PRESET, CALL HERE SO THE MANUAL COMMAND ENDS
  }
}
