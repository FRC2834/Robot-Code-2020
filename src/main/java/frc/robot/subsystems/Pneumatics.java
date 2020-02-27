/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pneumatics extends SubsystemBase {

  // Declare the compressor
  public Compressor c;
  // Declare the double solenoid
  public DoubleSolenoid armSolenoid; // Arm solenoid
  // Declare the single solenoids
  public Solenoid ratchetSolenoid; // Ratchet solenoid

  /**
   * Creates a new Pneumatics.
   */
  public Pneumatics() {
    // Initialize the compressor
    c = new Compressor(Constants.PCMID);
    c.setClosedLoopControl(true);
    c.start();
    // Declare the double solenoid
    armSolenoid = new DoubleSolenoid(Constants.PCMID, Constants.armSolenoidForward, Constants.armSolenoidReverse);
    // Declare the single solenoids
    ratchetSolenoid = new Solenoid(Constants.PCMID, Constants.ratchetSolenoid);
    // Set default position
    armSolenoid.set(Value.kOff);
    ratchetSolenoid.set(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
