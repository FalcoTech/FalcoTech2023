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
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double ArmPos = RobotContainer.m_arm.GetArmEncoderPosition();
    // if (ArmPos < 0){
    //   RobotContainer.m_arm.MoveArm(-.3);
    // } else if (ArmPos > 0 && ArmPos < 1.25){
    //   RobotContainer.m_arm.MoveArm(-.25);
    // } else if (ArmPos > 1.25 && ArmPos < 1.3){
    //   RobotContainer.m_arm.MoveArm(0);
    // } else if (ArmPos > 1.3){
    //   RobotContainer.m_arm.MoveArm(.05);
    // }
    if (ArmPos < 1){
      RobotContainer.m_arm.MoveArm(-.3);
    } else if (ArmPos > 1 && ArmPos < 1.3){
      RobotContainer.m_arm.MoveArm(-.2);
    } else if (ArmPos > 1.3){
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
    return RobotContainer.CoPilot.getRightY() > .1 || RobotContainer.CoPilot.getRightY() < -.1 || RobotContainer.CoPilot.getPOV() == 0 || RobotContainer.CoPilot.getPOV() == 180 || RobotContainer.CoPilot.getPOV() == 270;
  }
}
