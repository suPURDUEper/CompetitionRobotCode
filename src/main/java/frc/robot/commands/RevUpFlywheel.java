// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class RevUpFlywheel extends CommandBase {
  Shooter shooter;
  private boolean finish = false;
  Timer timer;
  /** Creates a new DriveForwardTimed. */
  public RevUpFlywheel(Shooter st) {
    shooter = st;
    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setFlywheelSpeed(5000);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      // shooter.setFlywheelSpeed(ShuffleboardInfo.getInstance().getFlywheelSpeed().getDouble(0.0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      shooter.setFlywheelSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}
