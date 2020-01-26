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
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
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

    // Configure all motors
    configureMotors();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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
    for(VictorSPX motor : shooterFollowers) {
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
    // Config peak and nominal outputs and enable coast
    shooterMotor.configNominalOutputForward(Constants.motorNominal);
    shooterMotor.configNominalOutputReverse(Constants.motorNominal);
    shooterMotor.configPeakOutputForward(Constants.motorPeakF);
    shooterMotor.configPeakOutputForward(Constants.motorPeakR);
    shooterMotor.setNeutralMode(NeutralMode.Coast);
    for(VictorSPX motor : shooterFollowers) {
      motor.configNominalOutputForward(Constants.motorNominal);
      motor.configNominalOutputReverse(Constants.motorNominal);
      motor.configPeakOutputForward(Constants.motorPeakF);
      motor.configPeakOutputForward(Constants.motorPeakR);
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
    hoodMotor.setSensorPhase(true);
    hoodMotor.setSelectedSensorPosition(0);
    // Config peak and nominal outputs and enable brake
    hoodMotor.configNominalOutputForward(Constants.motorNominal);
    hoodMotor.configNominalOutputReverse(Constants.motorNominal);
    hoodMotor.configPeakOutputForward(Constants.motorPeakF);
    hoodMotor.configPeakOutputForward(Constants.motorPeakR);
    hoodMotor.setNeutralMode(NeutralMode.Brake);
    // Config position closed loop gains
    hoodMotor.selectProfileSlot(Constants.shooterSlotIDx, Constants.shooterPIDIDx);
    hoodMotor.config_kF(Constants.shooterSlotIDx, Constants.shooterkF);
    hoodMotor.config_kP(Constants.shooterSlotIDx, Constants.shooterkP);
    hoodMotor.config_kI(Constants.shooterSlotIDx, Constants.shooterkI);
    hoodMotor.config_kD(Constants.shooterSlotIDx, Constants.shooterkD);

    // Configure turret motor
    turretMotor.configFactoryDefault();
    // Set direction
    turretMotor.setInverted(false);
    // Config relative encoder and zero it
    turretMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    turretMotor.setSensorPhase(true);
    turretMotor.setSelectedSensorPosition(0);
    // Config relevant frame periods to be at least as fast as periodic rate
		turretMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, Constants.turretPeriod);
		turretMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, Constants.turretPeriod);
    // Config peak and nominal outputs and enable brake
    turretMotor.configNominalOutputForward(Constants.motorNominal);
    turretMotor.configNominalOutputReverse(Constants.motorNominal);
    turretMotor.configPeakOutputForward(Constants.motorPeakF);
    turretMotor.configPeakOutputForward(Constants.motorPeakR);
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
   * Gets the translation vector from the smart dashboard.
   * @return The translation vector from the smart dashboard.
   */
  public double[] getTvec() {
    // Get the translation vector from the smart dashboard
    double x = SmartDashboard.getNumber("tvecX", 0.0);
    double y = SmartDashboard.getNumber("tvecY", 0.0);
    double z = SmartDashboard.getNumber("tvecZ", 0.0);
  
    return new double[] {x, y, z};
  }

  /**
   * Rotates the translation vector from the camera's angle to 0 radians/facing forward.
   * @param tvec The translation vector of the camera in the order of x, y, and z.
   * @param cameraAngleOfElevation The angle of elevation of the camera.
   * @return The translation vector relative to the camera, but at 0 radians/facing forward in the order of x, y, and z.
   */
  public double[] rotateTvec(double[] tvec, double cameraAngleOfElevation) {
    // Get the necessary values from tvec
    double y = tvec[1];
    double z = tvec[2];
    // Get the hypotenuse of y and z
    double hypot = Math.hypot(y, z);
    // Get the arctangent of y and z
    double theta = Math.atan(y / z);
    // Get the y to the target relative to the camera at 0 radians/facing forward
    double y2 = hypot * Math.sin(theta + cameraAngleOfElevation);
    // Get the z to the target relative to the camera at 0 radians/facing forward
    double z2 = hypot * Math.cos(theta + cameraAngleOfElevation);

    return new double[] {tvec[0], y2, z2};
  }

  /**
   * Translates the translation vector from the camera's position facing forward to the center of the fly wheel assembly.
   * @param tvecs The translation vector of the camera facing forward in the order of x, y, and z.
   * @param deltaX The amount to translate the x value.
   * @param deltaY The amount to translate the y value.
   * @param deltaZ The amount to translate the z value.
   * @return The translation vector relative to the center of the flywheel assembly in the order of x, y, and z.
   */
  public double[] translateTvec(double[] tvec, double deltaX, double deltaY, double deltaZ) {
    // Get the necessary values from tvec
    double x = tvec[0];
    double y = tvec[1];
    double z = tvec[2];
    // Translate each value
    x += deltaX;
    y += deltaY;
    z += deltaZ;

    return new double[] {x, y, z};
  }

  /**
   * Gets the horizontal distance to the target.
   * @param tvec The translation vector relative to the center of the flywheel assembly in the order of x, y, and z.
   * @return The horizontal distance to the target.
   */
  public double getDistanceToTarget(double[] tvec) {
    // Get the necessary values from tvec
    double x = tvec[0];
    double z = tvec[2];
    // Get the distance to the target
    double d = Math.hypot(x, z);

    return d;
  }

  /**
   * Gets the yaw to the target in radians.
   * @param tvec The translation vector relative to the center of the flywheel assembly in the order of x, y, and z.
   * @return The yaw to the target in radians.
   */
  public double getYawToTarget(double[] tvec) {
    // Get the necessary values from tvec
    double x = tvec[0];
    double z = tvec[2];
    // Get the yaw to the target
    double theta = Math.atan(x / z);

    return theta;
  }
  
  /**
   * Calculates the desired hood angle for the shooter at the current position of the robot.
   * @param tvec The translation vector relative to the center of the flywheel assembly in the order of x, y, and z.
   * @return The target angle of the hood in radians.
   */
  public double calculateHoodAngle(double[] tvec) {
    // Get the necessary values from tvec
    double y = tvec[1];
    double z = tvec[2];
    // Get the pitch to the target.
    double theta = Math.atan(y / z);
    // Get the hood angle
    double hoodAngle = ((Math.PI / 2) + theta) / 2;

    return hoodAngle;
  }

  /**
   * Calculates the initial velocity of the ball required to shoot at into the target.
   * @param tvec The translation vector relative to the center of the flywheel assembly in the order of x, y, and z.
   * @param hoodAngle The target angle of the hood in radians.
   * @return The target velocity of the ball in meters per second.
   */
  public double calculateInitialVelocity(double[] tvec, double hoodAngle) {

  }

  /**
   * Set the shooter motors to a certain velocity based on encoder ticks per 100 ms.
   * @param unitsPer100ms Encoder ticks per 100 ms.
   */
  public void setShooterVelocity(int unitsPer100ms) {
    shooterMotor.set(ControlMode.Velocity, unitsPer100ms);
  }
  
  /**
   * Set the hood motor to a certain encoder tick.
   * @param positionInTicks Target in encoder ticks.
   */
  public void setHoodPosition(int positionInTicks) {
    hoodMotor.set(ControlMode.Position, positionInTicks);
  }

  /**
   * Set the turret motor to a certain encoder tick.
   * @param positionInTicks Target in encoder ticks.
   */
  public void setTurretPosition(int positionInTicks) {
  }

}
