// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.armpresets;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.commands.RunArm;
import frc.robot.subsystems.Arm;

public class HighNodeArm extends CommandBase {
  /** Creates a new ScoreLowNodeArm. */
  public HighNodeArm() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.  
  @Override
  public void execute() {
    double ArmPos = RobotContainer.m_arm.GetArmEncoderPosition();
    if (ArmPos < 1.3){ //too far back
      RobotContainer.m_arm.MoveArm(-.3);
    } else if (ArmPos > 1.3 && ArmPos < 1.55){ //almost there
      if (RobotContainer.m_arm.GetArmExtended()){ //if extended, more force needed to hold
        RobotContainer.m_arm.MoveArm(-.25);
      } else{
        RobotContainer.m_arm.MoveArm(-.15);
      }
    } else if (ArmPos > 1.55){
      RobotContainer.m_arm.StopArm();
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
    return RobotContainer.CoPilot.getRightY() > .1 || RobotContainer.CoPilot.getRightY() < -.1 || RobotContainer.CoPilot.getPOV() == 90 || RobotContainer.CoPilot.getPOV() == 180 || RobotContainer.CoPilot.getPOV() == 270;
  }
}
