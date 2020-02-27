/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public interface Constants {
    // Drive motor controller IDs
    public static int leftFrontID = 1;
    public static int leftBackID = 2;
    public static int rightFrontID = 3;
    public static int rightBackID = 4;

    // Shooter motor IDs
    public static int shooterID0 = 5;
    public static int shooterID1 = 6;
    public static int shooterID2 = 7;
    public static int shooterID3 = 8;
    
    // Hood motor ID
    public static int hoodID = 9;

    // Turret motor ID
    public static int turretID = 10;

    // Intake motor ID
    public static int intakeID = 11;

    // Ball manager IDs
    public static int carouselID = 12;
    public static int feedID = 13;

    // Climber motor IDs
    public static int climberMotorLeft = 14;
    public static int climberMotorRight = 15;

    // Pneumatics ID
    public static int PCMID = 17;

    // General motor config constants
    public static int motorNominal = 0;
    public static int motorPeakF = 1;
    public static int motorPeakR = -1;

    // Shooter motor config constants
    public static int shooterSlotIDx = 0;
    public static int shooterPIDIDx = 0;
    public static double shooterkF = 0.95 * 1023 / 86128;
    public static double shooterkP = 0.04;
    public static double shooterkI = 0.0;
    public static double shooterkD = 0.0;

    // Hood motor config constants
    public static int hoodPeriod = 10;
    public static int hoodSlotIDx = 0;
    public static int hoodPIDIDx = 0;
    public static double hoodkF = 0.95 * 1023 / 1145;
    public static double hoodkP = 0.7;
    public static double hoodkI = 0.0;
    public static double hoodkD = 0.0;
    public static int hoodCruiseVelocity = 1145;
    public static int hoodAcceleration = 1145 * 4;

    // Turret motor config constants
    public static int turretPeriod = 10;
    public static int turretPIDIDx = 0;
    public static int turretSlotIDx = 0;
    public static double turretkF = 0.95 * 1023 / 2249;
    public static double turretkP = 0.125 * 1023 / (0.15 / (2 * Math.PI) * 20480);
    public static double turretkI = 0.00031;
    public static double turretkD = 225 * turretkP;
    public static int turretCruiseVelocity = 2249;
    public static int turretAcceleration = 2249 * 2;

    // Carousel motor config constants
    public static int carouselPeriod = 10;
    public static int carouselPIDIDx = 0;
    public static int carouselSlotIDx = 0;
    public static double carouselkF = 0.0;
    public static double carouselkP = 0.0;
    public static double carouselkI = 0.0;
    public static double carouselkD = 0.0;
    public static int carouselCruiseVelocity = 0;
    public static int carouselAcceleration = 0;

    // Shooter ticks per revolution
    public static double flywheelTicksPerRevolution = 4096 * 2.25;
    public static double hoodTicksPerRevolution = 4096 * (370 / 18);
    public static double turretTicksPerRevolution = 4096 * (202 / 36);

    // Hood zero angle
    public static double hoodZeroAngle = 11 * (Math.PI / 180);
    public static double hoodZeroTicks = hoodZeroAngle * (hoodTicksPerRevolution / (2 * Math.PI));

    // Drive ticks per revolution
    public static double driveTicksPerRevolution = 42;

    // Shooter speed multiplier
    public static double shooterVMultiplier = 1.0;

    // Buttons
    // Intake
    public static int intakeButton = 11;
    public static double intakePower = 0.95;
    // Output
    public static int outputButton = 2;
    public static double outputPower = -0.95;
    // Feed
    public static int feedButton = 3;
    public static double feedPower = 0.95;
    // Arm
    public static int armButton = 12;
    public static Value armValue = Value.kReverse;
    // Climber up
    public static int climberUpButton = 7;
    // Climber down
    public static int climberDownButton = 8;
    // Climb mode
    public static int climbModeButton = 6;

    // Current limits
    // Turret current limit
    public static int turretCurrentLimit = 50;
    // DriveTrain current limit
    public static int driveTrainCurrentLimit = 50;

    // DriveTrain open loop ramp rate
    public static double driveTrainRampRate = 0.3;

    // Max error for shooting
    public static double maxTurretYawError = 0.05;
    public static double maxHoodTickError = 227.5555555556;
    public static double maxFlywheelRPMError = 50;

    // PCM solenoid channel
    public static int armSolenoidForward = 2;
    public static int armSolenoidReverse = 1;
    public static int ratchetSolenoid = 0;

    // Climber parameters
    public static double climberUpSpeed = 0.5;
    public static double climberDownSpeed = -0.95;
    public static double climbUpDelay = 0.25;
    public static double climbUpDelay2 = climbUpDelay + 0.01;
    public static double climberTicksPerRevolution = 42.0;
    public static int climberHighTick = 4500;
    public static int climberLowTick = 0;
}
