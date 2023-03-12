// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.armpresets.ScoringPosArm;
import frc.robot.commands.autos.PlaceAndBalance.BalanceOnChargeStation;
import frc.robot.commands.autos.PlaceAndBalance.DriveToChargeStation;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceBalanceFullAuto extends SequentialCommandGroup {
  /** Creates a new BalanceFullAuto. */
  public PlaceBalanceFullAuto() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ScoringPosArm(),
      
      new DriveToChargeStation(),
      new BalanceOnChargeStation()
    );
  }
}
