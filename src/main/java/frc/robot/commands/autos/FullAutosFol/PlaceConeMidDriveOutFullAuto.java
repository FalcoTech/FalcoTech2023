// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.FullAutosFol;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autos.DriveBackwards;
import frc.robot.commands.autos.DriveForwards;
import frc.robot.commands.autos.DriveOut;
import frc.robot.commands.autos.LowerArmFromMidAuto;
import frc.robot.commands.autos.MidNodeAuto;
import frc.robot.commands.autos.Spit;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceConeMidDriveOutFullAuto extends SequentialCommandGroup {
  /** Creates a new PlaceConeMidDriveOutFullAuto. */
  public PlaceConeMidDriveOutFullAuto() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new DriveBackwards(), new MidNodeAuto(), new DriveForwards(), new LowerArmFromMidAuto(), new ParallelCommandGroup(new Spit(), new DriveOut())); //spit AND drive out
  }
}
