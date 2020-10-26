/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  // Declare differential drive
  public DifferentialDrive drive;

  // NavX Gyro
  public AHRS gyro;

  // Odometry class for tracking robot pose
  public final DifferentialDriveOdometry m_odometry;

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
    // Set to factory default
    leftDrive[0].restoreFactoryDefaults();
    leftDrive[1].restoreFactoryDefaults();
    rightDrive[0].restoreFactoryDefaults();
    rightDrive[1].restoreFactoryDefaults();
    // Set direction
    leftDrive[0].setInverted(true);
    // leftDrive[1].setInverted(false);
    rightDrive[0].setInverted(false);
    // rightDrive[1].setInverted(true);
    // Set idle mode
    leftDrive[0].setIdleMode(IdleMode.kBrake);
    leftDrive[1].setIdleMode(IdleMode.kBrake);
    rightDrive[0].setIdleMode(IdleMode.kBrake);
    rightDrive[1].setIdleMode(IdleMode.kBrake);
    // Set followers
    leftDrive[1].follow(leftDrive[0]);
    rightDrive[1].follow(rightDrive[0]);
    // Set open loop ramp rate
    leftDrive[0].setOpenLoopRampRate(Constants.driveTrainRampRate);
    leftDrive[1].setOpenLoopRampRate(Constants.driveTrainRampRate);
    rightDrive[0].setOpenLoopRampRate(Constants.driveTrainRampRate);
    rightDrive[1].setOpenLoopRampRate(Constants.driveTrainRampRate);
    // Set smart current limit
    leftDrive[0].setSmartCurrentLimit(Constants.driveTrainCurrentLimit);
    leftDrive[1].setSmartCurrentLimit(Constants.driveTrainCurrentLimit);
    rightDrive[0].setSmartCurrentLimit(Constants.driveTrainCurrentLimit);
    rightDrive[1].setSmartCurrentLimit(Constants.driveTrainCurrentLimit);
    // Initialize encoders
    leftEncoder = new CANEncoder(leftDrive[0]);
    rightEncoder = new CANEncoder(rightDrive[0]);
    leftEncoder.setPositionConversionFactor(Constants.metersPerRevolution);
    rightEncoder.setPositionConversionFactor(Constants.metersPerRevolution);
    leftEncoder.setVelocityConversionFactor(Constants.metersPerRevolution / 60); //rpm to meters per second
    rightEncoder.setVelocityConversionFactor(Constants.metersPerRevolution / 60);

    leftEncoder.setPosition(0.0);
    rightEncoder.setPosition(0.0);

    drive = new DifferentialDrive(leftDrive[0], rightDrive[0]);

    // Gyro instantiation
    gyro = new AHRS(Port.kMXP);
    gyro.zeroYaw();

    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(Rotation2d.fromDegrees(getHeading()), leftEncoder.getPosition(),
                      rightEncoder.getPosition());

    SmartDashboard.putNumber("gyro heading", getHeading());
    SmartDashboard.putNumber("gyro heading 2", gyro.getYaw());
    SmartDashboard.putNumber("left pos", leftEncoder.getPosition());
    SmartDashboard.putNumber("right pos", rightEncoder.getPosition());
    SmartDashboard.putNumber("left vel", leftEncoder.getVelocity());
    SmartDashboard.putNumber("right vel", rightEncoder.getVelocity());
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(), rightEncoder.getVelocity());
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Math.IEEEremainder(-gyro.getYaw(), 360);
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
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
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftDrive[0].setVoltage(leftVolts);
    rightDrive[0].setVoltage(rightVolts);
    drive.feed();
  }

  public void arcadeDrive(double power, double turn) {
    double leftPower = power - turn;
    double rightPower = power + turn;

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

    setVoltage(-leftPower*10, -rightPower*10);
  }
}
