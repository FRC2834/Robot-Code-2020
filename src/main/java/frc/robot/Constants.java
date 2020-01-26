/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public interface Constants {
    // Drive motor controller IDs
    public static int leftFrontID = 0;
    public static int leftBackID = 1;
    public static int rightFrontID = 2;
    public static int rightBackID = 3;

    // Shooter motor IDs
    // public static int shooterID0 = 4;
    // public static int shooterID1 = 5;
    // public static int shooterID2 = 6;
    // public static int shooterID3 = 7;
    public static int shooterID0 = 1;
    public static int shooterID1 = 2;
    public static int shooterID2 = 3;
    public static int shooterID3 = 4;
    
    // Hood motor ID
    public static int hoodID = 8;

    // Turret motor ID
    public static int turretID = 9;

    // Ball manager ID(s)
    
    // Intake motor ID

    // General motor config constants
    public static int motorNominal = 0;
    public static int motorPeakF = 1;
    public static int motorPeakR = -1;

    // Shooter motor config constants
    public static int shooterSlotIDx = 0;
    public static int shooterPIDIDx = 0;
    public static double shooterkF = 0.0;
    public static double shooterkP = 0.0;
    public static double shooterkI = 0.0;
    public static double shooterkD = 0.0;

    // Hood motor config constants
    public static int hoodSlotIDx = 0;
    public static int hoodPIDIDx = 0;
    // public static double hoodkF = 0.0;
    public static double hoodkP = 0.0;
    public static double hoodkI = 0.0;
    public static double hoodkD = 0.0;

    // Turret motor config constants
    public static int turretPeriod = 10;
    public static int turretPIDIDx = 0;
    public static int turretSlotIDx = 0;
    public static double turretkF = 0.0;
    public static double turretkP = 0.0;
    public static double turretkI = 0.0;
    public static double turretkD = 0.0;
    public static int turretCruiseVelocity = 0;
    public static int turretAcceleration = 0;

    // Camera position constants
    public static double cameraAngleOfElevationl = 0;
}
