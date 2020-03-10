/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.AimTurret;
import frc.robot.commands.ControlFeeder;
import frc.robot.commands.WaitForLock;
import frc.robot.commands.WaitForTime;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class CenterAuto extends SequentialCommandGroup {
  /**
   * Creates a new CenterAuto.
   */
  public CenterAuto(Shooter shooter, Feeder feeder) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(new ParallelDeadlineGroup(
      new SequentialCommandGroup(
        new WaitForLock(), 
        new ControlFeeder(feeder, Constants.feedPower), 
        new WaitForTime(5), 
        new ControlFeeder(feeder, Constants.feedNeutralPower)),
      new AimTurret(shooter)));
  }
}
