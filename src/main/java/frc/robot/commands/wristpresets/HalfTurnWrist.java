// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.wristpresets;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.commands.RunWrist;

public class HalfTurnWrist extends CommandBase {
  /** Creates a new FloorPickupWrist. */
  public HalfTurnWrist() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double WristEncoderPos = RobotContainer.m_wrist.GetWristEncoderPosition();

    RobotContainer.m_wrist.SetWristToPoint(WristEncoderPos, 200);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_wrist.TurnWrist(0);
    // RobotContainer.m_wrist.ResetWristEncoder();
    RobotContainer.m_wrist.setDefaultCommand(new RunWrist());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (RobotContainer.m_wrist.GetWristEncoderPosition() > 180 && RobotContainer.m_wrist.GetWristEncoderPosition() < 220) || RobotContainer.CoPilot.getLeftX() > .9 || RobotContainer.CoPilot.getLeftX() < -.9;
  }
}
