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
    double[] cameraTvec = subsystem.getTvec();
    double[] rotatedTvec = subsystem.rotateTvec(cameraTvec, Constants.cameraAngleOfElevation);
    double[] flywheelTvec = subsystem.translateTvec(rotatedTvec, Constants.deltaX, Constants.deltaY, Constants.deltaZ);
    double yaw = subsystem.getYawToTarget(flywheelTvec);
    double hoodTarget = subsystem.calculateHoodAngle(flywheelTvec);
    double[] hoodTranslation = subsystem.calculateHoodTranslation(hoodTarget, Constants.flywheelRadius, Constants.ballRadius);
    double[] hoodTvec = subsystem.translateTvec(flywheelTvec, 0, hoodTranslation[0], hoodTranslation[1]);
    double distanceToTarget = subsystem.getDistanceToTarget(hoodTvec);
    double idealInitV = subsystem.calculateIdealInitialVelocity(hoodTvec, distanceToTarget, hoodTarget, Constants.motionConstants.g);
    double correctedInitV = subsystem.calculateCorrectedVelocity(idealInitV, hoodTarget, Constants.ballRadius, Constants.motionConstants, Constants.targetHeight - hoodTvec[1], Constants.t, Constants.vStep, distanceToTarget, Constants.targetHeight);
    double RPM = subsystem.initVToRPM(correctedInitV, Constants.flywheelRadius);
    double flywheelTicks = subsystem.RPMToTicksPerSecond(RPM, Constants.flywheelTicksPerRevolution);
    int hoodTick = subsystem.getHoodTicksToTargetAngle(hoodTarget, Constants.hoodTicksPerRevolution);
    int turretTick = subsystem.getTurretTicksToTargetAngle(yaw, Constants.turretTicksPerRevolution);
    subsystem.shooterMotor.set(ControlMode.Velocity, flywheelTicks / 10);
    subsystem.hoodMotor.set(ControlMode.MotionMagic, hoodTick);
    subsystem.turretMotor.set(ControlMode.MotionMagic, turretTick);
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
