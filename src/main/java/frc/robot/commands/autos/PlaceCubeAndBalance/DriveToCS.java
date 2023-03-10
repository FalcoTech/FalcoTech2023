// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.PlaceCubeAndBalance;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class DriveToCS extends CommandBase {
  /** Creates a new DriveToChargeStation. */
  public DriveToCS() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double RobotYaw = RobotContainer.m_drivetrain.GetGyroYaw();
    if (RobotYaw > 2){
      RobotContainer.m_drivetrain.ArcadeDrive(-.2, .05);
    } else if (RobotYaw < 2){
      RobotContainer.m_drivetrain.ArcadeDrive(-.2, -.05);
    } else{
      RobotContainer.m_drivetrain.ArcadeDrive(-.2, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (RobotContainer.m_drivetrain.GetGyroPitch() > 15);
  }
}
