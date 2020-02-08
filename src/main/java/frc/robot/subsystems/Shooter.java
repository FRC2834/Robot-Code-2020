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
import frc.robot.MotionConstants;

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
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("turret encoder", turretMotor.getSelectedSensorPosition());
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
      motor.follow(shooterMotor);
      motor.set(ControlMode.Follower, 1);
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
    shooterMotor.configPeakOutputReverse(Constants.motorPeakR);
    shooterMotor.setNeutralMode(NeutralMode.Coast);
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
    hoodMotor.setSensorPhase(true);
    hoodMotor.setSelectedSensorPosition(0);
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
    turretMotor.configPeakOutputReverse(Constants.motorPeakR);
    turretMotor.setNeutralMode(NeutralMode.Brake);
    // Config neutral deadband
    turretMotor.configNeutralDeadband(0.001);
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
   * Calculates the translation from where the ball leaves the shooter relative to the center of the flyweel assembly.
   * @param hoodAngle The target angle of the hood in radians.
   * @param flywheelRadius The radius of the flywheel.
   * @param ballRadius The radius of the ball.
   * @return The translation from where the ball leaves the shooter relative to the center of the flyweel assembly.
   */
  public double[] calculateHoodTranslation(double hoodAngle, double flywheelRadius, double ballRadius) {
    // Get the y translation
    double deltaY = (flywheelRadius+ballRadius) * Math.sin(hoodAngle);
    // Get the z translation
    double deltaZ = (flywheelRadius+ballRadius) * Math.cos(hoodAngle);

    return new double[] {deltaY, deltaZ};
  }

  /**
   * Calculates the ideal initial velocity of the ball required to shoot at into the target.
   * @param tvec The translation vector at the point where the ball leaves the shooter in the order of x, y, and z.
   * @param distanceToTarget The distance to the target.
   * @param hoodAngle The target angle of the hood in radians.
   * @param g Acceleration due to gravity in meters per second^2.
   * @return The ideal initial velocity of the ball in meters per second.
   */
  public double calculateIdealInitialVelocity(double[] tvec, double distanceToTarget, double hoodAngle, double g) {
    // Get the necessary values from tvec
    double y = tvec[1];
    // Get the ideal initial velocity
    double a = Math.pow(distanceToTarget, 2) * g;
    double b = distanceToTarget * Math.sin(2 * hoodAngle) - 2 * y * Math.pow(Math.cos(hoodAngle), 2);
    double idealInitialVelocity = Math.sqrt(a / b);

    return idealInitialVelocity;
  }

  /**
   * Calculates the intial velocity x and velocity y.
   * @param initialVelocity The initial velocity of the ball in meters per second.
   * @param hoodAngle The target angle of the hood in radians
   * @return Initial velocity x and velocity y.
   */
  public double[] calculateInitialVxAndVy(double initialVelocity, double hoodAngle) {
    // Get Vx and Vy
    double InitVx = initialVelocity * Math.cos(hoodAngle);
    double InitVy = initialVelocity * Math.sin(hoodAngle);

    return new double[] {InitVx, InitVy};
  }

  /**
   * Calculates the angular velocity of the ball
   * @param initialVelocity The initial velocity of the ball in meters per second.
   * @param ballRadius The radius of the ball.
   * @return The angular velocity of the ball in radians per second.
   */
  public double calculateAngularVelocity(double initialVelocity, double ballRadius) {
    // Get angular velocity
    double tangentialVelocity = initialVelocity * 2;
    double w = tangentialVelocity / ballRadius;

    return w;
  }

  /**
   * Calculates acceleration x and acceleration y.
   * @param Cd The drag coefficient.
   * @param Ks The proportionality constant.
   * @param w Angular velocity in radians per second.
   * @param p Density of medium in kg per meters^3.
   * @param A Projected area in meters^2.
   * @param m Mass of object in kg.
   * @param g Acceleration due to gravity in meters per second^2.
   * @param Vx Velocity x in meters per second.
   * @param Vy Velocity y in meters per second.
   * @return Acceleration x and acceleration y.
   */
  public double[] calculateAxAndAy(double Cd, double Ks, double w, double p, double A, double m, double g, double Vx, double Vy) {
    // Calculate Ax and Ay
    double Ax = -0.5 * Cd * p * A * Vx * Math.sqrt(Math.pow(Vx, 2) + Math.pow(Vy, 2)) * (1 / m) - Ks * w * Vy * (1 / m);
    double Ay = -0.5 * Cd * p * A * Vy * Math.sqrt(Math.pow(Vx, 2) + Math.pow(Vy, 2)) * (1 / m) - g + Ks * w * Vx * (1 / m);

    return new double[] {Ax, Ay};
  }

  /**
   * 
   * @param initAx Initial acceleration x.
   * @param initAy Initial acceleration y.
   * @param initVx Initial velocity x.
   * @param initVy Initial velocity y.
   * @param initX Initial x.
   * @param initY Initial y.
   * @param w Angular velocity in radians per second.
   * @param t Timestep.
   * @param distanceToTarget Distance to target in meters
   * @param targetHeight Height of target in meters
   * @param constants Constants for caluclating Ax and Ay.
   * @return The height of the ball when it crosses the target plane.
   */
  public double calculateHeightAtTarget(double initAx, double initAy, double initVx, double initVy, double initX, double initY, double w, double t, double distanceToTarget, MotionConstants constants) {
    // Get initial values
    double[] currentAxAndAy = new double[] {initAx, initAy};
    double currentVx = initVx;
    double currentVy = initVy;
    double currentX = initX;
    double currentY = initY;

    while(currentX < distanceToTarget) {
      // Integrate over time
      currentX = currentVx * t + currentX;
      currentY = currentVy * t + currentY;
      currentVx = currentAxAndAy[0] * t + currentVx;
      currentVy = currentAxAndAy[1] * t + currentVy;
      currentAxAndAy = calculateAxAndAy(constants.Cd, constants.Ks, w, constants.p, constants.A, constants.m, constants.g, currentVx, currentVy);
    }

    return currentY;
  }

  /**
   * Calculates the velocity required to intercept the target.
   * @param idealInitialVelocity The ideal initial velocity of the ball in meters per second.
   * @param hoodAngle The angle of the hood in radians.
   * @param ballRadius The radius of the ball in meters.
   * @param constants The constants required for calculating accleration x and acceleration y.
   * @param initY The initial y position of the ball (i.e. when it leaves the shooter).
   * @param t The timestep in seconds.
   * @param vStep The amount velocity is incremented in meters per second to find the velocity required to intercept the target.
   * @param distanceToTarget The distance to the target in meters.
   * @param targetHeight The height of the target in meters
   * @return The velocity required to intercept the target in meters per second.
   */
  public double calculateCorrectedVelocity(double idealInitialVelocity, double hoodAngle, double ballRadius, MotionConstants constants, double initY, double t, double vStep, double distanceToTarget, double targetHeight) {
    // Initial velocity to start checking with.
    double initialVelocity = idealInitialVelocity;
    // Get Vx and Vy
    double[] VxAndVy = calculateInitialVxAndVy(initialVelocity, hoodAngle);
    // Get w
    double w = calculateAngularVelocity(initialVelocity, ballRadius);
    // Get Ax and Ay
    double[] AxAndAy = calculateAxAndAy(constants.Cd, constants.Ks, w, constants.p, constants.A, constants.m, constants.g, VxAndVy[0], VxAndVy[1]);
    // Get height at target
    double yAtTarget = calculateHeightAtTarget(AxAndAy[0], AxAndAy[1], VxAndVy[0], VxAndVy[1], 0, initY, w, t, distanceToTarget, constants);
    // Search higher or lower velocity
    if(yAtTarget > targetHeight) {
      while(yAtTarget > targetHeight) {
        // Increment initial velocity
        initialVelocity -= vStep;
        // Get Vx and Vy
        VxAndVy = calculateInitialVxAndVy(initialVelocity, hoodAngle);
        // Get w
        w = calculateAngularVelocity(initialVelocity, ballRadius);
        // Get Ax and Ay
        AxAndAy = calculateAxAndAy(constants.Cd, constants.Ks, w, constants.p, constants.A, constants.m, constants.g, VxAndVy[0], VxAndVy[1]);
        // Get height at target
        yAtTarget = calculateHeightAtTarget(AxAndAy[0], AxAndAy[1], VxAndVy[0], VxAndVy[1], 0, initY, w, t, distanceToTarget, constants);
      }
    } else if(yAtTarget < targetHeight) {
      while(yAtTarget < targetHeight) {
        // Increment initial velocity
        initialVelocity += vStep;
        // Get Vx and Vy
        VxAndVy = calculateInitialVxAndVy(initialVelocity, hoodAngle);
        // Get w
        w = calculateAngularVelocity(initialVelocity, ballRadius);
        // Get Ax and Ay
        AxAndAy = calculateAxAndAy(constants.Cd, constants.Ks, w, constants.p, constants.A, constants.m, constants.g, VxAndVy[0], VxAndVy[1]);
        // Get height at target
        yAtTarget = calculateHeightAtTarget(AxAndAy[0], AxAndAy[1], VxAndVy[0], VxAndVy[1], 0, initY, w, t, distanceToTarget, constants);
      }
    }

    return initialVelocity;
  }

  /**
   * Converts initial velocity of the ball to rotations per minute of the flywheel.
   * @param initialVelocity Initial velocity of the ball in meters per second.
   * @param flywheelRadius Radius of the flywheel in meters.
   * @return The rotations per minute of the flywheel.
   */
  public double initVToRPM(double initialVelocity, double flywheelRadius) {
    // Get tangential velocity of the ball and shooter
    double tangentialVelocity = initialVelocity * 2;
    // Get the RPM of the flywheel
    double RPM = tangentialVelocity / (2 * Math.PI * flywheelRadius) * 60;

    return RPM;
  }

  /**
   * Converts rotations per minute to encoder ticks per second.
   * @param RPM Rotation per minute.
   * @param ticksPerRevolution Encoder ticks per wheel revolution.
   * @return Encoder ticks per second.
   */
  public double RPMToTicksPerSecond(double RPM, double ticksPerRevolution) {
    // Get the ticks per second
    double ticksPerSecond = RPM / 60 * ticksPerRevolution;

    return ticksPerSecond;
  }

  /**
   * Moves the hood to the target angle.
   * @param targetAngle The target angle for the hood in radians.
   * @param ticksPerRevolution The ticks per revolution of the hood about the flywheel.
   * @return The target in encoder ticks.
   */
  public int getHoodTicksToTargetAngle(double targetAngle, double ticksPerRevolution) {
    // Get the encoder tick
    int targetTick = (int) (targetAngle * (ticksPerRevolution / (2 * Math.PI)));
    // Move the hood
    return targetTick;
  }

  /**
   * Moves the turret to the target angle.
   * @param yawToTarget The angle to move the hood to the target in radians.
   * @param ticksPerRevolution The ticks per revolution of the turret.
   * @return The target in encoder ticks.
   */
  public int getTurretTicksToTargetAngle(double yawToTarget, double ticksPerRevolution) {
    // Get the delta / yaw to target in ticks
    int delta = (int) (yawToTarget * (ticksPerRevolution / (2 * Math.PI)));
    // Get the turret's current position
    int currentPosition = turretMotor.getSelectedSensorPosition();
    // Move the turret
    //setTurretPosition(currentPosition + delta);
    return currentPosition + delta;
  }
}
