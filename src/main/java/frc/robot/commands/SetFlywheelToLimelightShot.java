// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;


public class SetFlywheelToLimelightShot extends CommandBase {
  private final Shooter shooter;
  private final Vision vision;
  /** Creates a new ShootBall. */
  public SetFlywheelToLimelightShot(Shooter mShooter, Vision vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    shooter = mShooter;
    this.vision = vision;
    addRequirements(mShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.enableShooter();
    shooter.setDistanceHoodPosition();
    shooter.setFlywheelDistanceRPM(vision.getTy());
    shooter.setAcceleratorDistanceRPM(vision.getTy());
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
}
