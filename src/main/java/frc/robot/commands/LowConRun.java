// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LowerConveyor;

public class LowConRun extends CommandBase {
  /** Creates a new LowConRun. */
  LowerConveyor lowCon;
  public LowConRun(LowerConveyor mLowerConveyor) {
    // Use addRequirements() here to declare subsystem dependencies.
    lowCon = mLowerConveyor;
    addRequirements(mLowerConveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lowCon.setLowerConveyorPercentOutput(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    lowCon.setLowerConveyorPercentOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
