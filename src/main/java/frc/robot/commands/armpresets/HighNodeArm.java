// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.armpresets;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class HighNodeArm extends CommandBase {
  private static double armposition;
  private static Timer armTimer = new Timer();

  /** Creates a new HighNodeArm. */
  public HighNodeArm() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armTimer.reset();
    armTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armposition = RobotContainer.m_arm.GetArmEncoderDegrees();

    if (armposition < 80){
      RobotContainer.m_arm.MoveArm(-.4);
    } else if (armposition >= 80 && armposition <107){
      RobotContainer.m_arm.MoveArm(-.2);      
    } else{
      RobotContainer.m_arm.StopArm();
    }
    //TODO add feedforward for arm extension
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_arm.StopArm();
    armTimer.stop();
    armTimer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (armposition > 107 && armposition < 110) || RobotContainer.CoPilotArmOverride() || armTimer.get() > 3;
  }
}
