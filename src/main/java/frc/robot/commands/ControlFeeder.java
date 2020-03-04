/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallManager;

public class ControlFeeder extends CommandBase {

  BallManager ballManager;
  double power;

  /**
   * Creates a new ControlFeeder.
   */
  public ControlFeeder(BallManager ballManager, double power) {
    this.ballManager = ballManager;
    this.power = power;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.ballManager);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ballManager.feedMotor.set(power);
    end(false);
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
