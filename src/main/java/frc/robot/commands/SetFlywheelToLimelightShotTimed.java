// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;


public class SetFlywheelToLimelightShotTimed extends CommandBase {
  private final Shooter shooter;
  private final Vision vision;
  private Timer timer = new Timer();
  private double time;
  /** Creates a new ShootBall. */
  public SetFlywheelToLimelightShotTimed(Shooter mShooter, Vision vision, double timeToRev) {
    // Use addRequirements() here to declare subsystem dependencies.
    shooter = mShooter;
    this.vision = vision;
    addRequirements(mShooter);
    time = timeToRev;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.enableShooter();
    shooter.setDistanceHoodPosition();
    shooter.setFlywheelDistanceRPM(vision.getTy());
    shooter.setAcceleratorDistanceRPM(vision.getTy());
    timer.reset();
    timer.start();
  }
  
  @Override
  public void execute() {
    shooter.setFlywheelDistanceRPM(vision.getTy());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.disableShooter();
  }

  @Override
  public boolean isFinished() {
    return (timer.get() > time);
  }
}
