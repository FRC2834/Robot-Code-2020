/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class Drive extends CommandBase {
  private final DriveTrain driveTrain;
  private final XboxController controller;
  /**
   * Creates a new Drive.
   */
  public Drive(DriveTrain driveTrain, XboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
    this.controller = controller;
    addRequirements(this.driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double power;
    double turn;

    if(Math.abs(controller.getY(Hand.kLeft)) < 0.1) {
      power = 0.0;
    } else {
      power = controller.getY(Hand.kLeft) * 0.5;
    }

    if(Math.abs(controller.getX(Hand.kRight)) < 0.1) {
      turn = 0.0;
    } else {
      turn = controller.getX(Hand.kRight) * 0.5;
    }
    driveTrain.arcadeDrive(power, turn);
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
