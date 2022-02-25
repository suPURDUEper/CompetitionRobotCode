// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;


public class SetFlywheelToFarShot extends InstantCommand {
  private final Shooter shooter;
  /** Creates a new ShootBall. */
  public SetFlywheelToFarShot(Shooter mShooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    shooter = mShooter;
    addRequirements(mShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setDistanceHoodPosition();
    shooter.setFlywheelTargetRPM(Constants.Shooter.SHOOTER_FAR_SHOT_RPM);
  }
}
