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

public class TurretToSetpoint extends CommandBase {

  private Shooter shooter;
  private double turretAngle;
  private double hoodAngle;
  private double rpm;
  private double targetTick;
  private double hoodAngleTick;
  private double shooterTicksPer100Ms;

  /**
   * Creates a new TurretToSetpoint.
   */
  public TurretToSetpoint(Shooter shooter, double turretAngle, double hoodAngle, double rpm) {
    this.shooter = shooter;
    this.turretAngle = turretAngle;
    this.hoodAngle = hoodAngle;
    this.rpm = rpm;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetTick = shooter.getTurretYawTick(turretAngle * 180 / Math.PI, Constants.turretTicksPerRevolution);
    hoodAngleTick = shooter.getHoodTargetTick(hoodAngle, Constants.hoodTicksPerRevolution);
    shooterTicksPer100Ms = shooter.getShooterTicksPer100Ms(rpm, Constants.flywheelTicksPerRevolution);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.turretMotor.set(ControlMode.MotionMagic, targetTick);
    shooter.hoodMotor.set(ControlMode.MotionMagic, hoodAngleTick);
    if(Math.abs(targetTick - shooter.turretMotor.getSelectedSensorPosition()) < Constants.flywheelActivationThreshold) {
      shooter.shooterMotor.set(ControlMode.Velocity, shooterTicksPer100Ms);
      end(false);
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
