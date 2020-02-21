/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class AimTurret extends CommandBase {
  // The subsystem the command runs on
  private final Shooter subsystem;

  /**
   * Creates a new AimTurret.
   * @param subsystem The subsystem used by this command.
   */
  public AimTurret(Shooter subsystem) {
    this.subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(SmartDashboard.getBoolean("Target Detected?", false)) {
      double ticksToTarget = subsystem.getTurretYawTick(SmartDashboard.getNumber("turretYawError", 0.0), Constants.turretTicksPerRevolution);
      subsystem.turretMotor.set(ControlMode.MotionMagic, subsystem.turretMotor.getSelectedSensorPosition() - ticksToTarget);
      
      double hoodAngleTicks = subsystem.getHoodTargetTick(SmartDashboard.getNumber("targetHoodAngle", Math.PI / 4), Constants.hoodTicksPerRevolution);
      subsystem.hoodMotor.set(ControlMode.MotionMagic, hoodAngleTicks - Constants.hoodZeroTicks);

      //double shooterTicksPer100Ms = subsystem.getShooterTicksPer100Ms(SmartDashboard.getNumber("shooterV (rads/sec)", 0.0) , Constants.flywheelTicksPerRevolution) * Constants.shooterVMultiplier;
      // double shooterTicksPer100Ms = SmartDashboard.getNumber("rpm", 0.0) / 600 * Constants.flywheelTicksPerRevolution;
      double rpm = subsystem.getClosestRPM(SmartDashboard.getNumber("distance", 0.0));
      double shooterTicksPer100Ms = rpm / 600 * Constants.flywheelTicksPerRevolution;
      SmartDashboard.putNumber("shooter 100 Ms", shooterTicksPer100Ms);
      subsystem.shooterMotor.set(ControlMode.Velocity, shooterTicksPer100Ms);

      SmartDashboard.putBoolean("Tracking?", true);
      SmartDashboard.putNumber("Target Tick", subsystem.turretMotor.getSelectedSensorPosition() - ticksToTarget);

      SmartDashboard.putNumber("Hood Target Tick", hoodAngleTicks);
      SmartDashboard.putNumber("Hood Tick", subsystem.hoodMotor.getSelectedSensorPosition());
      SmartDashboard.putNumber("Hood Target Angle Deg", SmartDashboard.getNumber("targetHoodAngle", 0) * (180 / Math.PI));

      SmartDashboard.putNumber("Target velocity", shooterTicksPer100Ms * 10 / Constants.flywheelTicksPerRevolution * 60);
      SmartDashboard.putNumber("Shooter velocity", subsystem.shooterMotor.getSelectedSensorVelocity() * 10 / Constants.flywheelTicksPerRevolution * 60);
    } else {
      subsystem.shooterMotor.set(ControlMode.Velocity, 0.0);
      SmartDashboard.putBoolean("Tracking?", false);
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
