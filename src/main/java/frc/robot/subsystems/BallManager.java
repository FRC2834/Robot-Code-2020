/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BallManager extends SubsystemBase {

  // Declare motors
  public TalonSRX carouselMotor;
  public CANSparkMax feedMotor;

  /**
   * Creates a new BallManager.
   */
  public BallManager() {
    // Initialize motors
    carouselMotor = new TalonSRX(Constants.carouselID);
    feedMotor = new CANSparkMax(Constants.feedID, MotorType.kBrushless);

    // Configure motors
    carouselMotor.configFactoryDefault();
    feedMotor.restoreFactoryDefaults();
    // Set directions
    carouselMotor.setInverted(true);
    feedMotor.setInverted(false);
    // Set idle mode
    feedMotor.setIdleMode(IdleMode.kBrake);
    // Config absolute encoder
    carouselMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    carouselMotor.setSensorPhase(true);
    // Config relevant frame periods to be at least as fast as periodic rate
		carouselMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, Constants.carouselPeriod);
		carouselMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, Constants.carouselPeriod);
    // Config peak and nominal outputs and enable brake
    carouselMotor.configNominalOutputForward(Constants.motorNominal);
    carouselMotor.configNominalOutputReverse(Constants.motorNominal);
    carouselMotor.configPeakOutputForward(Constants.motorPeakF);
    carouselMotor.configPeakOutputReverse(Constants.motorPeakR);
    carouselMotor.setNeutralMode(NeutralMode.Brake);
    // Config position closed loop gains
    carouselMotor.selectProfileSlot(Constants.carouselSlotIDx, Constants.carouselPIDIDx);
    carouselMotor.config_kF(Constants.carouselSlotIDx, Constants.carouselkF);
    carouselMotor.config_kP(Constants.carouselSlotIDx, Constants.carouselkP);
    carouselMotor.config_kI(Constants.carouselSlotIDx, Constants.carouselkI);
    carouselMotor.config_kD(Constants.carouselSlotIDx, Constants.carouselkD);
    // Config acceleration and cruise velocity
		carouselMotor.configMotionCruiseVelocity(Constants.carouselCruiseVelocity);
    carouselMotor.configMotionAcceleration(Constants.carouselAcceleration);
    
    SmartDashboard.putNumber("Carousel Power", 0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Carousel Current", carouselMotor.getStatorCurrent());
  }
}
