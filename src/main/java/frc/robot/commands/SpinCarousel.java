/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.BallManager;

public class SpinCarousel extends CommandBase {

  BallManager ballManager;
  boolean isJammed;
  double startTime;

  /**
   * Creates a new SpinCarousel.
   * @param ballManager The ball manager subsystem
   */
  public SpinCarousel(BallManager ballManager) {
    this.ballManager = ballManager;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.ballManager);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isJammed = false;
    SmartDashboard.putBoolean("Carousel Jammed?", false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Checks if the carousel is jammed
    if(ballManager.carouselMotor.getBusVoltage() >= Constants.jamCurrent) {
      isJammed = true;
      startTime = Timer.getFPGATimestamp();
      SmartDashboard.putBoolean(("Carousel Jammed?"), true);
    }

    // Handle the jam by reversing the carousel
    if(isJammed) {
      if(Timer.getFPGATimestamp() - startTime <= Constants.pauseDuration) {
        if(Timer.getFPGATimestamp() - startTime <= Constants.unjamDuration) {
          ballManager.carouselMotor.set(ControlMode.PercentOutput, Constants.unjamPower);
        } else {
          ballManager.carouselMotor.set(ControlMode.PercentOutput, 0.0);
        }
      } else {
        isJammed = false;
        SmartDashboard.putBoolean("Carousel Jammed?", false);
      }
    } else {
      ballManager.carouselMotor.set(ControlMode.PercentOutput, Constants.carouselPower);
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
