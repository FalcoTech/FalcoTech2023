// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.armpresets;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.commands.RunArm;
import frc.robot.subsystems.Arm;

public class MidNodeArm extends CommandBase {
  /** Creates a new ScoreLowNodeArm. */
  public MidNodeArm() {
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
    double ArmPos = RobotContainer.m_arm.GetArmEncoderPosition();

    if (ArmPos < 1){ //too far back
      RobotContainer.m_arm.MoveArm(-.3);
    } else if (ArmPos > 1 && ArmPos < 1.35){ //almost there
      RobotContainer.m_arm.MoveArm(-.2);
    } else if (ArmPos > 1.5){ //too high
      RobotContainer.m_arm.MoveArm(.05);; //might need to run back
    } else{ //hold?
      RobotContainer.m_arm.MoveArm(-.05);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_arm.setDefaultCommand(new RunArm());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.CoPilot.getRightY() > .1 || RobotContainer.CoPilot.getRightY() < -.1
    || RobotContainer.CoPilot.getPOV() == 0 || RobotContainer.CoPilot.getPOV() == 180 || RobotContainer.CoPilot.getPOV() == 270 || RobotContainer.CoPilot.getBackButton();
  }
}
