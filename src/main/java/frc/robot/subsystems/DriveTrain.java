/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  
  // Declare left drive motors
  public CANSparkMax[] leftDrive;
  // Declare right drive motors
  public CANSparkMax[] rightDrive;
  // Declare the left encoder
  public CANEncoder leftEncoder;
  // Declare the right encoder
  public CANEncoder rightEncoder;

  private static Talon driveLeftFront;
  private static Talon driveLeftBack;
  private static Talon driveRightFront;
  private static Talon driveRightBack; 

  /**
   * Creates a new DriveTrain.
   */
  public DriveTrain() {

    // Initialize motors
    leftDrive = new CANSparkMax[] {
      new CANSparkMax(Constants.leftFrontID, MotorType.kBrushless),
      new CANSparkMax(Constants.leftBackID, MotorType.kBrushless)
    };
    rightDrive = new CANSparkMax[] {
      new CANSparkMax(Constants.rightFrontID, MotorType.kBrushless),
      new CANSparkMax(Constants.rightBackID, MotorType.kBrushless)
    };
    // Set direction
    leftDrive[0].setInverted(false);
    leftDrive[1].setInverted(false);
    rightDrive[0].setInverted(false);
    rightDrive[1].setInverted(false);
    // Set followers
    leftDrive[1].follow(leftDrive[0]);
    rightDrive[1].follow(rightDrive[0]);
    // Initialize encoders
    leftEncoder = new CANEncoder(leftDrive[0]);
    rightEncoder = new CANEncoder(rightDrive[0]);
    leftEncoder.setPositionConversionFactor(Constants.driveTicksPerRevolution);
    rightEncoder.setPositionConversionFactor(Constants.driveTicksPerRevolution);

    driveLeftFront = new Talon(0);
    driveLeftBack = new Talon(3);
    driveRightFront = new Talon(2);
    driveRightBack = new Talon(1);

    driveLeftFront.setInverted(false);
    driveLeftBack.setInverted(false);
    driveRightFront.setInverted(true);
    driveRightBack.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   * 
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void setVoltage(double leftVolts, double rightVolts) {
    leftDrive[0].setVoltage(leftVolts);
    rightDrive[0].setVoltage(rightVolts);
  }

  public void setPower(double left, double right) {
    leftDrive[0].set(left);
    rightDrive[0].set(right);
    driveLeftFront.set(left);
    driveLeftBack.set(left);
    driveRightFront.set(right);
    driveRightBack.set(right);
  }

  public void arcadeDrive(double power, double turn) {
    double leftPower = power + turn;
    double rightPower = power - turn;

    double max = Math.abs(leftPower);
    if(Math.abs(rightPower) > max) {
      max = Math.abs(rightPower);
    }

    if(max > 1.0) {
      leftPower /= max;
      rightPower /= max;
    }

    if(leftPower > 0 ) {
      leftPower = -Math.pow((leftPower - 1), 2) + 1;
    } else {
      leftPower = -(-Math.pow((Math.abs(leftPower) - 1), 2) + 1);
    }

    if(rightPower > 0 ) {
      rightPower = -Math.pow((rightPower - 1), 2) + 1;
    } else {
      rightPower = -(-Math.pow((Math.abs(rightPower) - 1), 2) + 1);
    }

    setPower(leftPower, rightPower);
  }
}
