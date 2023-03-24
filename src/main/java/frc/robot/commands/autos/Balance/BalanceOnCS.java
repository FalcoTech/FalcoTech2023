// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.Balance;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class BalanceOnCS extends CommandBase {
  private double chargingspeed = -.25;
  /** Creates a new BalanceOnChargeStation. */
  public BalanceOnCS() {
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
    
    if (RobotPitch > 3){
      chargingspeed -= .005;
      RobotContainer.m_drivetrain.ArcadeDrive(chargingspeed, 0);
    } else if (RobotPitch < -3){
      chargingspeed += .005;
      RobotContainer.m_drivetrain.ArcadeDrive(chargingspeed, 0);
    } else {
      if (chargingspeed > 0){
        chargingspeed -= .005;
      }
      else if (chargingspeed < 0){
        chargingspeed += .005;
      }
      RobotContainer.m_drivetrain.ArcadeDrive(chargingspeed, 0);
    }

    if (chargingspeed >= .2){
      chargingspeed = .2;
    } else if (chargingspeed <= -.2){
      chargingspeed = -.2;
    }
  }
    

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_drivetrain.CoastDriveMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
