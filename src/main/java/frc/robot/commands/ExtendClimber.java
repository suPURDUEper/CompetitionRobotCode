// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Pneumatics;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ExtendClimber extends CommandBase {
  public final Pneumatics pneumatics;
  /** Creates a new ExtendClimber. */
  public ExtendClimber(Pneumatics mPneumatics) {
    // Use addRequirements() here to declare subsystem dependencies.
    pneumatics = mPneumatics;
    addRequirements(mPneumatics);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
