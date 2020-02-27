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

public class ControlCarousel extends CommandBase {

  BallManager subsystem;
  double power;
  boolean isJammed;
  double startTime;

  /**
   * Creates a new ControlCarousel.
   */
  public ControlCarousel(BallManager subsystem, double power) {
    this.subsystem = subsystem;
    this.power = power;
    isJammed = false;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(subsystem.carouselMotor.getStatorCurrent() >= Constants.jamCurrent) {
      isJammed = true;
      startTime = Timer.getFPGATimestamp();
      SmartDashboard.putBoolean("Carousel Jammed?", true);
    }
    
    if(isJammed) {
      if((Timer.getFPGATimestamp() - startTime) <= Constants.unjamDuration) {
        subsystem.carouselMotor.set(ControlMode.PercentOutput, Constants.unjamPower);
      } else {
        isJammed = false;
      }
    } else {
      subsystem.carouselMotor.set(ControlMode.PercentOutput, Constants.carouselPower);
      SmartDashboard.putBoolean("Carousel Jammed?", false);
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
