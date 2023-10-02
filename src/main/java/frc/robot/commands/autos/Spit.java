// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class Spit extends CommandBase {
  /** Creates a new Spit. */
  private static Timer spitTimer = new Timer();
  public Spit() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    spitTimer.reset();
    spitTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.m_intake.RunIntake(.4); //multiply by negative if running wrong way

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    spitTimer.stop();
    spitTimer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return spitTimer.get() > 4;
  }
}
