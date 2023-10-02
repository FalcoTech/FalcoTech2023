// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class LowerArmFromMidAuto extends CommandBase {
  /** Creates a new LowerArmFromMidAuto. */

  private static Timer TooLowTimer = new Timer();
  private static double armposition;
  public LowerArmFromMidAuto() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    TooLowTimer.reset();
    TooLowTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armposition = RobotContainer.m_arm.GetArmEncoderDegrees();

    RobotContainer.m_arm.MoveArm(.1);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_arm.StopArm();

    TooLowTimer.stop();
    TooLowTimer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (armposition < 80) || TooLowTimer.get() > 5;
  }
}
