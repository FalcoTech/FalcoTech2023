// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.armpresets;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.RunArm;
import frc.robot.commands.wristpresets.ZeroWrist;

public class ZeroArm extends CommandBase {
  public static Timer armTimer = new Timer();
  
  /** Creates a new ZeroArm. */
  public ZeroArm() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.m_arm.RetractArm();
    // new ZeroWrist();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double ArmPos = RobotContainer.m_arm.GetArmEncoderPosition();
    if (ArmPos < -.3){
      RobotContainer.m_arm.MoveArm(-.3);
    } else if (ArmPos > -.3 && ArmPos < -.1){
      RobotContainer.m_arm.MoveArm(-.125);
    } else if (ArmPos > .3){
      RobotContainer.m_arm.MoveArm(.3);
    } else if (ArmPos < .3 && ArmPos > .1){
      RobotContainer.m_arm.MoveArm(.125);
    } else {
      RobotContainer.m_arm.StopArm();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_arm.StopArm();
    RobotContainer.m_arm.setDefaultCommand(new RunArm()); //set default command back to user control when command finishes
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.CoPilot.getRightY() > .1 || RobotContainer.CoPilot.getRightY() < -.1 
    || RobotContainer.CoPilot.getPOV() == 0 || RobotContainer.CoPilot.getPOV() == 90 || RobotContainer.CoPilot.getPOV() == 270 || RobotContainer.CoPilot.getBackButton();
  }
}
