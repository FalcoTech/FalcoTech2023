// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.armpresets;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.commands.RunArm;

public class GroundPickupArm extends CommandBase {
  /** Creates a new GroundPickupArm. */
  public GroundPickupArm() {
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
    if (ArmPos > -.3){
      RobotContainer.m_arm.MoveArm(.3);
    } else if (ArmPos < -.3 && ArmPos > -.5){
      RobotContainer.m_arm.MoveArm(.2);
    } else if (ArmPos < -.5 && ArmPos > -.6){
      RobotContainer.m_arm.MoveArm(.125);
    } else{
    }
    if (ArmPos < -.9){
      RobotContainer.m_arm.MoveArm(-.3);
    } else if (ArmPos > -.9 && ArmPos < -.7){
      RobotContainer.m_arm.MoveArm(-.2);
    } else if (ArmPos > -.7 && ArmPos < -.6){
      RobotContainer.m_arm.MoveArm(-.125);
    } else{
    }
    if (ArmPos > -.625 && ArmPos < -.575){
      RobotContainer.m_arm.ExtendArm();
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
    || RobotContainer.CoPilot.getPOV() == 0 || RobotContainer.CoPilot.getPOV() == 90 || RobotContainer.CoPilot.getPOV() == 180 || RobotContainer.CoPilot.getPOV() == 270;
  }
}