// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
import java.util.function.DoubleSupplier;

public class ArcadeDrive extends CommandBase {
  // private final DriveTrain m_drivetrain;

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
    double PilotLeftY = (RobotContainer.Pilot.getLeftY() * -1); 
    double PilotRightX = RobotContainer.Pilot.getRightX();
    
    if (RobotContainer.m_drivetrain.arcadeDriveSpeed == "default"){
      RobotContainer.m_drivetrain.arcadeDrive(PilotLeftY, PilotRightX);
    } else {
      RobotContainer.m_drivetrain.arcadeDrive(PilotLeftY*.2, PilotRightX*.2);
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
