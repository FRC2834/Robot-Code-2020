/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Pneumatics;

public class Climb extends CommandBase {

  Climber climber;
  Pneumatics pneumatics;
  public enum Direction {
    UP,
    DOWN
  }
  Direction direction;
  boolean leftRatchetReleased;
  boolean rightRatchetReleased;
  double startTime;
  Joystick buttonBox;

  /**
   * Creates a new Climb.
   */
  public Climb(Climber climber, Pneumatics pneumatics, Direction direction, Joystick buttonBox) {
    this.climber = climber;
    this.pneumatics = pneumatics;
    this.direction = direction;
    this.buttonBox = buttonBox;
    leftRatchetReleased = false;
    rightRatchetReleased = false;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.climber, this.pneumatics);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    leftRatchetReleased = false;
    rightRatchetReleased = false;
    if(buttonBox.getRawButton(Constants.climbModeButton)) {
      // Get the start time of the command
      startTime = Timer.getFPGATimestamp();
      switch(direction) {
        case UP:
          // Extend the pancake piston
          pneumatics.ratchetSolenoid.set(true);
          break;
        case DOWN:
          break;
      }
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(buttonBox.getRawButton(Constants.climbModeButton)) {
      switch(direction) {
        case UP:
          if(climber.climberEncoderLeft.getPosition() <= Constants.climberHighTick) {
            if(leftRatchetReleased) {
              climber.climberMotorLeft.set(Constants.climberUpSpeed);
            } else {
              if(Timer.getFPGATimestamp() - startTime >= Constants.climbUpDelay) {
                climber.climberMotorLeft.set(-0.25);
                  if(Timer.getFPGATimestamp() - startTime >= Constants.climbUpDelay2) {
                    leftRatchetReleased = true;
                  }
              }
            }
          } else {
            climber.climberMotorLeft.set(0.0);
          }

          if(climber.climberEncoderRight.getPosition() <= Constants.climberHighTick) {
            if(rightRatchetReleased) {
              climber.climberMotorRight.set(Constants.climberUpSpeed);
            } else {
              if(Timer.getFPGATimestamp() - startTime >= Constants.climbUpDelay) {
                climber.climberMotorRight.set(-0.25);
                  if(Timer.getFPGATimestamp() - startTime >= Constants.climbUpDelay2) {
                    rightRatchetReleased = true;
                  }
              }
            }
          } else {
            climber.climberMotorRight.set(0.0);
          }
          break;
        case DOWN:
          if(climber.climberEncoderLeft.getPosition() > Constants.climberLowTick) {
            climber.climberMotorLeft.set(Constants.climberDownSpeed);
          } else {
            climber.climberMotorLeft.set(0.0);
          }

          if(climber.climberEncoderRight.getPosition() > Constants.climberLowTick) {
            climber.climberMotorRight.set(Constants.climberDownSpeed);
          } else {
            climber.climberMotorRight.set(0.0);
          }
          break;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pneumatics.ratchetSolenoid.set(false);
    climber.climberMotorLeft.set(0.0);
    climber.climberMotorRight.set(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
