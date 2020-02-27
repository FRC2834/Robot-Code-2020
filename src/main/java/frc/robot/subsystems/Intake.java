/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  // Declare intake motor
  public CANSparkMax intakeMotor;

  /**
   * Creates a new Intake.
   */
  public Intake() {
    // Initialize motor
    intakeMotor = new CANSparkMax(Constants.intakeID, MotorType.kBrushless);
    intakeMotor.restoreFactoryDefaults();
    // Set direction
    intakeMotor.setInverted(true);
    // Set idle mode
    intakeMotor.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
