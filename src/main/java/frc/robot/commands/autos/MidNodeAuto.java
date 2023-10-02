// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import java.sql.Time;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class MidNodeAuto extends CommandBase {
  private static Timer armTimer = new Timer();
  private static double armposition;
  /** Creates a new MidNodeAuto. */
  public MidNodeAuto() {
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

    if (armposition <= 75){
      RobotContainer.m_arm.MoveArm(-.4);
    } else if (armposition > 75 && armposition < 96){
      RobotContainer.m_arm.MoveArm(-.2);
    } else if (armposition >= 96){
      RobotContainer.m_arm.MoveArm(.1);
    } else {
      RobotContainer.m_arm.StopArm();
    }
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
    return (armposition > 96 && armposition < 98) || RobotContainer.CoPilotArmOverride() || armTimer.get() > 5;
  }
}
