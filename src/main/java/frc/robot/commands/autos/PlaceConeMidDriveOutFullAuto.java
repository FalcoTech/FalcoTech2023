// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.armpresets.MidNodeArm;
import frc.robot.commands.autos.PlaceConeDriveOut.DriveBackToGrid;
import frc.robot.commands.autos.PlaceConeDriveOut.DriveOffGrid;
import frc.robot.commands.autos.PlaceConeDriveOut.DriveOutOfCommunity;
import frc.robot.commands.autos.PlaceConeDriveOut.LowerArmMidNode;
import frc.robot.commands.autos.PlaceConeDriveOut.MidNodeArmAuto;
import frc.robot.commands.autos.PlaceConeDriveOut.Spit;
import frc.robot.commands.autos.PlaceConeDriveOut.ZeroArmAuto;
import frc.robot.commands.autos.PlaceConeDriveOut.groundpickupexperimental.GroundPickupArmAuto;
import frc.robot.commands.autos.PlaceConeDriveOut.groundpickupexperimental.TurnToPiece;
import frc.robot.commands.wristpresets.HalfTurnWrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceConeMidDriveOutFullAuto extends SequentialCommandGroup {
  /** Creates a new PlaceConeDriveOutAuto. */
  public PlaceConeMidDriveOutFullAuto() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(new DriveOffGrid(), new MidNodeArmAuto()),
      new DriveBackToGrid(),
      new LowerArmMidNode(),
      new ParallelDeadlineGroup(new DriveOffGrid(), new Spit()), //does DriveOffGrid need to go with spit command (again, not just in deadline param)?
      new ParallelCommandGroup(new ZeroArmAuto(), new DriveOutOfCommunity())
      // new ParallelCommandGroup(new GroundPickupArmAuto(), new TurnToPiece())
      // new HalfTurnWrist()
      );
  }
}
