// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;

public class IntakeStop extends InstantCommand {
  /** Creates a new IntakeStop. */
  private final Intake intake;
  public IntakeStop(Intake mIntake) {
    intake = mIntake;
    addRequirements(mIntake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.IntakeMotorSet(0);
    intake.IndexerMotorSet(0);
  }
}
