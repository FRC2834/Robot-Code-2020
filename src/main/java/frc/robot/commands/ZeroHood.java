/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class ZeroHood extends CommandBase {

  Shooter shooter;

  /**
   * Creates a new ZeroHood.
   */
  public ZeroHood(Shooter shooter) {
    this.shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(shooter.hoodMotor.getStatorCurrent() <= Constants.hoodJamCurrent) {
      shooter.hoodMotor.set(ControlMode.PercentOutput, 0.0);
      shooter.hoodMotor.setSelectedSensorPosition(0);
      end(false);
    } else {
      shooter.hoodMotor.set(ControlMode.PercentOutput, Constants.hoodZeroingPower);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
