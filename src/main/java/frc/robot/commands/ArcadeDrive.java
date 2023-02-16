// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;


public class ArcadeDrive extends CommandBase {


  /** Creates a new ArcadeDrive. */
  public ArcadeDrive() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double PilotLeftX = RobotContainer.Pilot.getLeftX();
    double PilotLeftY = RobotContainer.Pilot.getLeftY();
    double PilotRightX = RobotContainer.Pilot.getRightX();
    double PilotRightY = RobotContainer.Pilot.getRightY();

    double PilotRightTrigger = RobotContainer.Pilot.getRightTriggerAxis();
    double PilotLeftTrigger = RobotContainer.Pilot.getLeftTriggerAxis();
    double slowTriggerTurn = PilotRightX + (PilotRightTrigger*.1) - (PilotLeftTrigger*.1);

    if (RobotContainer.m_drivetrain.arcadeDriveSpeed == "default"){
      RobotContainer.m_drivetrain.ArcadeDrive(PilotLeftY, slowTriggerTurn);
    } else{
      RobotContainer.m_drivetrain.ArcadeDrive(PilotLeftY * .15, slowTriggerTurn * .15);
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
