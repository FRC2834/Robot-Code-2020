/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  // Declare shooter motors
  public TalonSRX shooterMotor;
  private VictorSPX[] shooterFollowers;

  // Declare hood motor
  private TalonSRX hoodMotor;

  // Declare turret motor
  private TalonSRX turretMotor;

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

    // Configure shooter motors
    shooterMotor.configFactoryDefault();
    for(VictorSPX motor : shooterFollowers) {
      motor.configFactoryDefault();
    }
    // Set followers
    // for(VictorSPX motor : shooterFollowers) {
    //   motor.follow(shooterMotor);
    // }
    // Set direction
    shooterMotor.setInverted(false);
    shooterFollowers[0].setInverted(InvertType.FollowMaster);
    shooterFollowers[1].setInverted(InvertType.OpposeMaster);
    shooterFollowers[2].setInverted(InvertType.OpposeMaster);
    // Config relative encoder
    shooterMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    shooterMotor.setSensorPhase(true);
    // Config peak and nominal outputs and enable coast
    shooterMotor.configNominalOutputForward(Constants.shooterNominal);
    shooterMotor.configNominalOutputReverse(Constants.shooterNominal);
    shooterMotor.configPeakOutputForward(Constants.shooterPeakF);
    shooterMotor.configPeakOutputForward(Constants.shooterPeakR);
    shooterMotor.setNeutralMode(NeutralMode.Coast);
    for(VictorSPX motor : shooterFollowers) {
      motor.configNominalOutputForward(Constants.shooterNominal);
      motor.configNominalOutputReverse(Constants.shooterNominal);
      motor.configPeakOutputForward(Constants.shooterPeakF);
      motor.configPeakOutputForward(Constants.shooterPeakR);
      motor.setNeutralMode(NeutralMode.Coast);
    }
    // Config velocity closed loop gains
    shooterMotor.config_kF(Constants.shooterPIDSlot, Constants.shooterkF);
    shooterMotor.config_kP(Constants.shooterPIDSlot, Constants.shooterkP);
    shooterMotor.config_kI(Constants.shooterPIDSlot, Constants.shooterkI);
    shooterMotor.config_kD(Constants.shooterPIDSlot, Constants.shooterkD);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setShooterVelocity(int unitsPer100ms) {
    shooterMotor.set(ControlMode.Velocity, unitsPer100ms);
  }

  public void setShooterPercent(double percent) {
    shooterMotor.set(ControlMode.PercentOutput, percent);
    SmartDashboard.putBoolean("running", true);
  }
}
