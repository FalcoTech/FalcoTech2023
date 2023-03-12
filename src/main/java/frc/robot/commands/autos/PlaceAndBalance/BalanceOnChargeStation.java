// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.PlaceAndBalance;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class BalanceOnChargeStation extends CommandBase {
  /** Creates a new BalanceOnChargeStation. */
  public BalanceOnChargeStation() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.m_drivetrain.BrakeDriveMotors();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double RobotPitch = RobotContainer.m_drivetrain.GetGyroPitch();
    double RobotPitchAccel = RobotContainer.m_drivetrain.GetGyroPitchAcceleration();

    if (RobotPitch > 3 && RobotPitchAccel > 5){
      RobotContainer.m_drivetrain.ArcadeDrive(-.05, 0);
    } else if (RobotPitch < -3 && RobotPitchAccel < -5){
      RobotContainer.m_drivetrain.ArcadeDrive(.05, 0);
    } else if (RobotPitchAccel > 5){
      RobotContainer.m_drivetrain.ArcadeDrive(.025, 0);
    } else if (RobotPitchAccel < -5){
      RobotContainer.m_drivetrain.ArcadeDrive(.025, 0);
    } else{
      RobotContainer.m_drivetrain.ArcadeDrive(0, 0);
      
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
