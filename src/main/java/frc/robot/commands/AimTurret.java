/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class AimTurret extends CommandBase {
  // The subsystem the command runs on
  Shooter shooter;
  Joystick buttonBox;

  /**
   * Creates a new AimTurret.
   * @param subsystem The subsystem used by this command.
   */
  public AimTurret(Shooter shooter, Joystick buttonBox) {
    this.shooter = shooter;
    this.buttonBox = buttonBox;
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
    // Check if the target is detected
    if(SmartDashboard.getBoolean("Target Detected?", false)) {
      // Point the turret at the target 
      double ticksToTarget = shooter.getTurretYawTick(SmartDashboard.getNumber("turretYawError", 0.0), Constants.turretTicksPerRevolution);
      double turretTargetTick = shooter.turretMotor.getSelectedSensorPosition() + ticksToTarget;
      if((ticksToTarget >= Constants.turretLowLimitTick) && (ticksToTarget <= Constants.turretHighLimitTick)) {
        shooter.turretMotor.set(ControlMode.MotionMagic, shooter.turretMotor.getSelectedSensorPosition() + ticksToTarget);
      }
      // Move the hood to the target angle
      double hoodAngleTicks = shooter.getHoodTargetTick(SmartDashboard.getNumber("hoodAngle (deg)", 45), Constants.hoodTicksPerRevolution);
      shooter.hoodMotor.set(ControlMode.MotionMagic, hoodAngleTicks);
      // Set the velocity of the flywheel
      double shooterTicksPer100Ms = shooter.getShooterTicksPer100Ms(SmartDashboard.getNumber("targetRPM", 0.0), Constants.flywheelTicksPerRevolution);
      if(Math.abs(ticksToTarget) < Constants.flywheelActivationThreshold) {
        shooter.shooterMotor.set(ControlMode.Velocity, shooterTicksPer100Ms);
      }

      // Set tracking status to true
      SmartDashboard.putBoolean("Tracking?", true);

      // Target tick of the turret
      SmartDashboard.putNumber("Target Turret Tick", turretTargetTick);
      // Target tick of the hood
      double hoodTargetTick = hoodAngleTicks;
      SmartDashboard.putNumber("Hood Target Tick", hoodTargetTick);
      // Target velocity of the flywheel
      double targetRPM = shooterTicksPer100Ms * 10 / Constants.flywheelTicksPerRevolution * 60;
      SmartDashboard.putNumber("Target RPM", targetRPM);

      // Get the errors
      double turretYawError = Math.abs(SmartDashboard.getNumber("turretYawError", 0.0));
      double hoodTickError = Math.abs(hoodTargetTick - shooter.hoodMotor.getSelectedSensorPosition());
      double flywheelRPMError = Math.abs(targetRPM - shooter.shooterMotor.getSelectedSensorVelocity() * 10 / Constants.flywheelTicksPerRevolution * 60);

      // Check if the shooter is locked on to the target
      if((turretYawError <= Constants.maxTurretYawError) && (hoodTickError <= Constants.maxHoodTickError) && (flywheelRPMError <= Constants.maxFlywheelRPMError)) {
        SmartDashboard.putBoolean("Target Locked", true);
      } else {
        SmartDashboard.putBoolean("Target Locked", false);
      }
    } else {
      // Turn off the flywheel
      shooter.shooterMotor.set(ControlMode.PercentOutput, 0.0);

      // Set tracking status to false
      SmartDashboard.putBoolean("Tracking?", false);
      SmartDashboard.putBoolean("Target Locked", false);
    }

    // Only set to setpoint if the setpoint button is pressed.
    // // Send the turret to a set point
    // subsystem.turretMotor.set(ControlMode.MotionMagic, SmartDashboard.getNumber("turret setpoint", 0.0));
    // // Send the hood to a set point
    // subsystem.hoodMotor.set(ControlMode.MotionMagic, SmartDashboard.getNumber("hood setpoint", 0.0));
    // // Set the RPM of the flywheel
    // subsystem.shooterMotor.set(ControlMode.Velocity, subsystem.getShooterTicksPer100Ms(SmartDashboard.getNumber("flywheel rpm", 0.0), Constants.flywheelTicksPerRevolution));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.shooterMotor.set(ControlMode.PercentOutput, 0.0);
    shooter.hoodMotor.set(ControlMode.PercentOutput, 0.0);
    shooter.turretMotor.set(ControlMode.PercentOutput, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
