// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LowerConveyor;
import frc.robot.subsystems.UpperConveyor;

public class Index extends CommandBase {
  /** Creates a new IntakeRun. */
  private final LowerConveyor lowCon;
  private final UpperConveyor upperCon;

  public Index(LowerConveyor mLowerConveyor, UpperConveyor mUpperConveyor) {
    // Use addRequirements() here to declare subsystem dependencies.
    lowCon = mLowerConveyor;
    addRequirements(mLowerConveyor);
    upperCon = mUpperConveyor;
    addRequirements(mUpperConveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!upperCon.hasTopBall()) {
      // We have no balls, run everything
      lowCon.setPooperPercentOutput(.8);
      lowCon.setLowerConveyorPercentOutput(1);
      upperCon.setPercentOutput(1);
    } else if (upperCon.hasTopBall() && !upperCon.hasTwoBalls()) {
      // We have one ball, run everything but upper con
      upperCon.setPercentOutput(0);
      lowCon.setPooperPercentOutput(.8);
      lowCon.setLowerConveyorPercentOutput(1);
    } 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    lowCon.setPooperPercentOutput(0);
    lowCon.setLowerConveyorPercentOutput(0);
    upperCon.setPercentOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return upperCon.hasTwoBalls();
  }
}
