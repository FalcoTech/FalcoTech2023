// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.wristpresets;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ZeroWrist extends CommandBase {
  private static Timer wristTimer = new Timer();
  private static double wristposition;
  /** Creates a new ZeroWrist. */
  public ZeroWrist() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wristTimer.reset();
    wristTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    wristposition = RobotContainer.m_wrist.GetWristEncoderDegrees();

    if (wristposition > 3){
      RobotContainer.m_wrist.TurnWrist(.5);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_wrist.StopWrist();
    wristTimer.stop();
    wristTimer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return wristposition < 3 || RobotContainer.CoPilotWristOverride() || wristTimer.get() > 4;
  }
}
