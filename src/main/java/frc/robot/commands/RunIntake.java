// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

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
    double IntakeTriggers = (CoPilotLeftTrigger - CoPilotRightTrigger); 

    RobotContainer.m_intake.RunIntake(IntakeTriggers); //multiply by negative if running wrong way
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
