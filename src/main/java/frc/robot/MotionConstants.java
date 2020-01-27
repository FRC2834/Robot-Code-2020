/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Add your docs here.
 */
public class MotionConstants {
    // Declare constants
    public final double Cd;
    public final double Ks;
    public final double p;
    public final double A;
    public final double m;
    public final double g;

    /**
     * 
     * @param Cd The drag coefficient.
     * @param Ks The proportionality constant.
     * @param p Density of medium in kg per meters^3.
     * @param A Projected area in meters^2.
     * @param m Mass of object in kg.
     * @param g Acceleration due to gravity in meters per second^2.
     * @return Acceleration x and acceleration y.
     */
    public MotionConstants(double Cd, double Ks, double p, double A, double m, double g) {
        this.Cd = Cd;
        this.Ks = Ks;
        this.p = p;
        this.A = A;
        this.m = m;
        this.g = g;
    }
}
