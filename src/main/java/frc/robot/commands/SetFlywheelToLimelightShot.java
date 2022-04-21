// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;


public class SetFlywheelToLimelightShot extends CommandBase {
  private final Shooter shooter;
  private final Vision vision;
  private MedianFilter filter;
  private double lastKnownTy; 
  /** Creates a new ShootBall. */
  public SetFlywheelToLimelightShot(Shooter mShooter, Vision vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    shooter = mShooter;
    this.vision = vision;
    addRequirements(mShooter);
    filter = new MedianFilter(10);
    lastKnownTy = 0.0;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.enableShooter();
    shooter.setDistanceHoodPosition();
    shooter.setFlywheelDistanceRPM(vision.getTy());
    shooter.setAcceleratorDistanceRPM(vision.getTy());
    filter.reset();
  }
  
  @Override
  public void execute() {
    if (vision.isTargetValid()) {
      double filteredTy = filter.calculate(vision.getTy());
      shooter.setFlywheelDistanceRPM(filteredTy);
      shooter.setAcceleratorDistanceRPM(filteredTy);
      lastKnownTy = filteredTy;
    } else {
      shooter.setFlywheelDistanceRPM(lastKnownTy);
      shooter.setFlywheelDistanceRPM(lastKnownTy);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.disableShooter();
  }
}
