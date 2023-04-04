// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.PlaceConeDriveOut;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.commands.RunArm;

public class MidNodeArmAuto extends CommandBase {
  /** Creates a new MidNodeArmAuto. */
  public MidNodeArmAuto() {
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

    if (ArmPos < 1){ //too far back
      RobotContainer.m_arm.MoveArm(-.3);
    } else if (ArmPos > 1 && ArmPos < 1.35){ //almost there
      RobotContainer.m_arm.MoveArm(-.175);
    } else if (ArmPos > 1.45){ //too high
      RobotContainer.m_arm.MoveArm(.1);; //might need to run back
    } else{ //hold?
      RobotContainer.m_arm.MoveArm(-.05);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_arm.StopArm();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.m_arm.GetArmEncoderPosition() > 1.35;
  }
}