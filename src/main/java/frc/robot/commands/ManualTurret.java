/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class ManualTurret extends CommandBase {

  Shooter shooter;
  XboxController controller;

  /**
   * Creates a new ManualTurret.
   */
  public ManualTurret(Shooter shooter, XboxController controller) {
    this.shooter = shooter;
    this.controller = controller;
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
    // Move the turret
    if(controller.getPOV() == 270) {
      shooter.turretMotor.set(ControlMode.PercentOutput, Constants.turretManualPower);
    } else if(controller.getPOV() == 90) {
      shooter.turretMotor.set(ControlMode.PercentOutput, -Constants.turretManualPower);
    } else if(controller.getPOV() == 0) {
      shooter.hoodMotor.set(ControlMode.PercentOutput, Constants.hoodManualPower);
    } else if(controller.getPOV() == 180) {
      shooter.hoodMotor.set(ControlMode.PercentOutput, -Constants.hoodManualPower);
    } else {
      shooter.turretMotor.set(ControlMode.PercentOutput, 0.0);
      shooter.hoodMotor.set(ControlMode.PercentOutput, 0.0);
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
