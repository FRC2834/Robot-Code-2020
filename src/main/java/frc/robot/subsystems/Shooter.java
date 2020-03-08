/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  
  // Declare shooter motors
  public TalonSRX shooterMotor;
  public VictorSPX[] shooterFollowers;

  // Declare hood motor
  public TalonSRX hoodMotor;

  // Declare turret motor
  public TalonSRX turretMotor;

  /**
   * Creates a new Shooter.
   */
  public Shooter() {
    // Initialize shooter motors
    shooterMotor = new TalonSRX(Constants.shooterID0);
    
    shooterFollowers = new VictorSPX[] {
      new VictorSPX(Constants.shooterID1),
      new VictorSPX(Constants.shooterID2),
      new VictorSPX(Constants.shooterID3)
    };

    // Initialize hood motor
    hoodMotor = new TalonSRX(Constants.hoodID);

    // Initialize turret motor
    turretMotor = new TalonSRX(Constants.turretID);

    // Configure all motors
    configureMotors();
  }

  @Override
  public void periodic() {
    // Current hood tick
    SmartDashboard.putNumber("Turret Tick", turretMotor.getSelectedSensorPosition());
    // Current hood tick
    SmartDashboard.putNumber("Hood Tick", hoodMotor.getSelectedSensorPosition());
    // Current flywheel velocity
    SmartDashboard.putNumber("Shooter RPM", shooterMotor.getSelectedSensorVelocity() * 10 / Constants.flywheelTicksPerRevolution * 60);
    // Hood current
    SmartDashboard.putNumber("Hood Current", hoodMotor.getStatorCurrent());
  }

  /**
   * Configures all motors
   */
  public void configureMotors() {
    // Configure shooter motors
    shooterMotor.configFactoryDefault();
    for(VictorSPX motor : shooterFollowers) {
      motor.configFactoryDefault();
    }
    // Set followers
    shooterMotor.set(ControlMode.Follower, 0);
    for(VictorSPX motor : shooterFollowers) {
      motor.set(ControlMode.Follower, 1);
      motor.follow(shooterMotor);
    }
    // Set direction
    shooterMotor.setInverted(false);
    shooterFollowers[0].setInverted(InvertType.FollowMaster);
    shooterFollowers[1].setInverted(InvertType.OpposeMaster);
    shooterFollowers[2].setInverted(InvertType.OpposeMaster);
    // Config relative encoder and zero it
    shooterMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    shooterMotor.setSensorPhase(true);
    shooterMotor.setSelectedSensorPosition(0);
    // Config peak and nominal outputs, enable coast, and current limit
    shooterMotor.configNominalOutputForward(Constants.motorNominal);
    shooterMotor.configNominalOutputReverse(Constants.motorNominal);
    shooterMotor.configPeakOutputForward(Constants.motorPeakF);
    shooterMotor.configPeakOutputReverse(Constants.motorPeakR);
    shooterMotor.setNeutralMode(NeutralMode.Coast);
    shooterMotor.configContinuousCurrentLimit(Constants.turretCurrentLimit);
    for(VictorSPX motor : shooterFollowers) {
      motor.configNominalOutputForward(Constants.motorNominal);
      motor.configNominalOutputReverse(Constants.motorNominal);
      motor.configPeakOutputForward(Constants.motorPeakF);
      motor.configPeakOutputReverse(Constants.motorPeakR);
      motor.setNeutralMode(NeutralMode.Coast);
    }
    // Config velocity closed loop gains
    shooterMotor.selectProfileSlot(Constants.shooterSlotIDx, Constants.shooterPIDIDx);
    shooterMotor.config_kF(Constants.shooterSlotIDx, Constants.shooterkF);
    shooterMotor.config_kP(Constants.shooterSlotIDx, Constants.shooterkP);
    shooterMotor.config_kI(Constants.shooterSlotIDx, Constants.shooterkI);
    shooterMotor.config_kD(Constants.shooterSlotIDx, Constants.shooterkD);

    // Configure hood motor
    hoodMotor.configFactoryDefault();
    // Set direction
    hoodMotor.setInverted(false);
    // Config relative encoder and zero it
    hoodMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    hoodMotor.setSensorPhase(false);
    hoodMotor.setSelectedSensorPosition((int) Constants.hoodZeroTicks);
    // Config relevant frame periods to be at least as fast as periodic rate
		hoodMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, Constants.hoodPeriod);
		hoodMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, Constants.hoodPeriod);
    // Config peak and nominal outputs and enable brake
    hoodMotor.configNominalOutputForward(Constants.motorNominal);
    hoodMotor.configNominalOutputReverse(Constants.motorNominal);
    hoodMotor.configPeakOutputForward(Constants.motorPeakF);
    hoodMotor.configPeakOutputReverse(Constants.motorPeakR);
    hoodMotor.setNeutralMode(NeutralMode.Brake);
    // Config position closed loop gains
    hoodMotor.selectProfileSlot(Constants.hoodSlotIDx, Constants.hoodPIDIDx);
    hoodMotor.config_kF(Constants.hoodSlotIDx, Constants.hoodkF);
    hoodMotor.config_kP(Constants.hoodSlotIDx, Constants.hoodkP);
    hoodMotor.config_kI(Constants.hoodSlotIDx, Constants.hoodkI);
    hoodMotor.config_kD(Constants.hoodSlotIDx, Constants.hoodkD);
    // Config acceleration and cruise velocity
		hoodMotor.configMotionCruiseVelocity(Constants.hoodCruiseVelocity);
		hoodMotor.configMotionAcceleration(Constants.hoodAcceleration);

    // Configure turret motor
    turretMotor.configFactoryDefault();
    // Set direction
    turretMotor.setInverted(false);
    // Config relative encoder and zero it
    turretMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    turretMotor.setSensorPhase(false);
    turretMotor.setSelectedSensorPosition(0);
    // Config relevant frame periods to be at least as fast as periodic rate
		turretMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, Constants.turretPeriod);
		turretMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, Constants.turretPeriod);
    // Config peak and nominal outputs and enable brake
    turretMotor.configNominalOutputForward(Constants.motorNominal);
    turretMotor.configNominalOutputReverse(Constants.motorNominal);
    turretMotor.configPeakOutputForward(Constants.motorPeakF);
    turretMotor.configPeakOutputReverse(Constants.motorPeakR);
    turretMotor.setNeutralMode(NeutralMode.Brake);
    // Config motion magic closed loop gains
    turretMotor.selectProfileSlot(Constants.turretSlotIDx, Constants.turretPIDIDx);
    turretMotor.config_kF(Constants.turretSlotIDx, Constants.turretkF);
    turretMotor.config_kP(Constants.turretSlotIDx, Constants.turretkP);
    turretMotor.config_kI(Constants.turretSlotIDx, Constants.turretkI);
    turretMotor.config_kD(Constants.turretSlotIDx, Constants.turretkD);
    // Config acceleration and cruise velocity
		turretMotor.configMotionCruiseVelocity(Constants.turretCruiseVelocity);
		turretMotor.configMotionAcceleration(Constants.turretAcceleration);
  }

  /**
   * Gets the number of encoder ticks to the target.
   * @param yaw The number of radians to the target.
   * @param encoderTicksPerRevolution The number of encoder ticks per revolution of the turret.
   * @return The number of encoder ticks to the target.
   */
  public double getTurretYawTick(double yaw, double encoderTicksPerRevolution) {
    double deltaTicks = (yaw / (2 * Math.PI)) * encoderTicksPerRevolution;
    
    return deltaTicks;
  }

  public double getHoodTargetTick(double hoodAngleDeg, double encoderTicksPerRevolution) {
    double targetTick = hoodAngleDeg / 360 * encoderTicksPerRevolution;

    return targetTick;
  }

  public double getShooterTicksPer100Ms(double RPM, double encoderTicksPerRevolution) {
    double targetTicksPer100Ms = RPM / 600 * encoderTicksPerRevolution;

    return targetTicksPer100Ms;
  }
}
