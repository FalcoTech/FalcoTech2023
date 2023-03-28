// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.autos.PlaceConeDriveOut.DriveOutOfCommunity;
import frc.robot.commands.autos.PlaceConeDriveOut.DriveToGrid;
import frc.robot.commands.autos.PlaceConeDriveOut.ScoringPosMidNode;
import frc.robot.commands.autos.PlaceConeDriveOut.ZeroArmInAuto;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceConeMidDriveOutAuto extends SequentialCommandGroup {
  /** Creates a new PlaceConeDriveOutAuto. */
  public PlaceConeMidDriveOutAuto() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ScoringPosMidNode(),
      new WaitCommand(1)
    );
  }
}
