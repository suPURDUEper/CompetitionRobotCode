// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class RevFlywheelToFenderShot extends CommandBase {
  private final Shooter shooter;

  /** Creates a new RevFlywheelToFenderShot. */
  public RevFlywheelToFenderShot(Shooter mShooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    shooter = mShooter;
    addRequirements(mShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setFenderHoodPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setFlywheelRPM(3000);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setFlywheelRPM(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
