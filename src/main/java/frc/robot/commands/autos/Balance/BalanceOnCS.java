// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.Balance;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;

public class BalanceOnCS extends CommandBase {
  private double prevPitch;

  /** Creates a new BalanceOnCS. */
  public BalanceOnCS() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double Pitch = RobotContainer.m_drivetrain.GetGyroPitch();

    if (Pitch <= AutoConstants.BALANCE_FULL_TILT){
      double error = Math.copySign(Math.abs(Pitch), Pitch);
      double power = Math.min(Math.abs(AutoConstants.BALANCE_KP * error), AutoConstants.BALANCE_MAX_POWER);
      power = Math.copySign(power, error) * -1;  //maybe 1?
  
      power /= (1 + (Math.abs(prevPitch - Pitch) * AutoConstants.BALANCE_DENOMINATOR_MULTIPLIER));

      RobotContainer.m_drivetrain.ArcadeDrive(power, 0);
    }
    
    prevPitch = Pitch;
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
