/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pneumatics;

public class ControlPneumatics extends CommandBase {

  Pneumatics pneumatics;
  public enum Solenoid {
    ARM_SOLENOID,
    RATCHET_SOLENOID
  }
  Solenoid solenoid;
  Value value;
  Boolean valueBoolean;

  /**
   * Creates a new ControlPneumatics.
   */
  public ControlPneumatics(Pneumatics pneumatics, Solenoid solenoid, Value value) {
    this.pneumatics = pneumatics;
    this.solenoid = solenoid;
    this.value = value;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.pneumatics);
  }

  /**
   * Creates a new ControlPneumatics.
   */
  public ControlPneumatics(Pneumatics pneumatics, Solenoid solenoid, Boolean value) {
    this.pneumatics = pneumatics;
    this.solenoid = solenoid;
    this.valueBoolean = value;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.pneumatics);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(solenoid) {
      case ARM_SOLENOID:
        pneumatics.armSolenoid.set(value);
        break;
      case RATCHET_SOLENOID:
        pneumatics.ratchetSolenoid.set(valueBoolean);
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
