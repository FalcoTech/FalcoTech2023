// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.armpresets;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.RunArm;

public class ZeroArm extends CommandBase {
  public static double ArmEncoderPos = RobotContainer.m_arm.GetArmEncoderPosition();
  
  /** Creates a new ZeroArm. */
  public ZeroArm() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (ArmEncoderPos >= -10){
      RobotContainer.m_arm.MoveArm(.2);
    } else if (ArmEncoderPos <= 10){
      RobotContainer.m_arm.MoveArm(-.2);
    } else {
      RobotContainer.m_arm.MoveArm(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_arm.MoveArm(0);
    RobotContainer.m_arm.setDefaultCommand(new RunArm()); //set default command back to user control when command finishes
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (ArmEncoderPos >= -10 && ArmEncoderPos <= 10);
  }
}
