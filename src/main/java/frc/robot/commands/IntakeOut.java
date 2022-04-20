// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;

public class IntakeOut extends InstantCommand {
  private final Intake intake;
  /** Creates a new IntakeOut. */
  public IntakeOut(Intake mIntake) {
    intake = mIntake;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.intakeOut();
    intake.IntakeMotorSet(1);
    intake.IndexerMotorSet(1);
  }
}
