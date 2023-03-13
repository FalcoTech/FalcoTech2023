// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.armpresets;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.RunArm;

public class ScoringPosArm extends CommandBase {
  /** Creates a new ScoringPosArm. */
  public ScoringPosArm() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.m_arm.RetractArm();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double ArmEncoderPos = RobotContainer.m_arm.GetArmEncoderPosition();

    RobotContainer.m_arm.SetArmToPoint(ArmEncoderPos, 200);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_arm.StopArm();
    // RobotContainer.m_arm.ExtendArm();
    RobotContainer.m_arm.setDefaultCommand(new RunArm());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((RobotContainer.m_arm.GetArmEncoderPosition() > 180 && RobotContainer.m_arm.GetArmEncoderPosition() < 220) || (RobotContainer.CoPilot.getRightY() > .9 || RobotContainer.CoPilot.getRightY() < -.9));
  }
}
