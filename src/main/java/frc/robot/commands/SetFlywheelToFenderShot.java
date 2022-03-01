// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Shooter;

public class SetFlywheelToFenderShot extends CommandBase {
  private final Shooter shooter;

  /** Creates a new RevFlywheelToFenderShot. */
  public SetFlywheelToFenderShot(Shooter mShooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    shooter = mShooter;
    addRequirements(mShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.enableShooter();
    shooter.setFenderHoodPosition();
    shooter.setFlywheelTargetRPM(3000);
  }

  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.disableShooter();
  }
}
