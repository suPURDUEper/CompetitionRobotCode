// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class SetFlywheelToLowShot extends CommandBase {
  /** Creates a new SetFlywheelToLowShot. */
  public final Shooter shooter;
  public SetFlywheelToLowShot(Shooter mShooter) {
    shooter = mShooter;
    addRequirements(mShooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.enableShooter();
    shooter.setAcceleratorTargetRPM(950);
    shooter.setFlywheelTargetRPM(950);
    shooter.setDistanceHoodPosition();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.disableShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
