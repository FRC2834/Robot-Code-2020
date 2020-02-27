/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

  // Declare climber motors
  public CANSparkMax climberMotorLeft;
  public CANSparkMax climberMotorRight;
  // Declare climber encoders
  public CANEncoder climberEncoderLeft;
  public CANEncoder climberEncoderRight;

  /**
   * Creates a new Climber.
   */
  public Climber() {
    // Initialize climber motors
    climberMotorLeft = new CANSparkMax(Constants.climberMotorLeft, MotorType.kBrushless);
    climberMotorRight = new CANSparkMax(Constants.climberMotorRight, MotorType.kBrushless);
    climberMotorLeft.restoreFactoryDefaults();
    climberMotorRight.restoreFactoryDefaults();
    // Set direction
    climberMotorLeft.setInverted(false);
    climberMotorRight.setInverted(false);
    // Set idle mode
    climberMotorLeft.setIdleMode(IdleMode.kBrake);
    climberMotorRight.setIdleMode(IdleMode.kBrake);
    // Initialize encoders
    climberEncoderLeft = new CANEncoder(climberMotorLeft);
    climberEncoderRight = new CANEncoder(climberMotorRight);
    climberEncoderLeft.setPositionConversionFactor(Constants.climberTicksPerRevolution);
    climberEncoderRight.setPositionConversionFactor(Constants.climberTicksPerRevolution);
    climberEncoderLeft.setPosition(0);
    climberEncoderRight.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Climber Pos", climberEncoderLeft.getPosition());
    SmartDashboard.putNumber("Right Climber Pos", climberEncoderRight.getPosition());
  }
}
