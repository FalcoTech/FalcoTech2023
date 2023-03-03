// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LEDs;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class BlinkGreen extends CommandBase {
  public static double step = 0; //if needed, put this in LEDs subsystem then check it from m_leds

  /** Creates a new BlinkGreen. */
  public BlinkGreen() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_leds);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    step = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    step += 1;
    if (step < 50){
      RobotContainer.m_leds.Green();
    } else if (step > 50 && step < 100){
      RobotContainer.m_leds.LEDOff();
    } else if (step > 100){
      step = 0;
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
