// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.lowerCon;
import frc.robot.subsystems.LowerConveyor;

public class LowerConStop extends InstantCommand {
  /** Creates a new LowerConStop. */
  private final LowerConveyor lowCon;
  public LowerConStop(LowerConveyor mLowCon) {
    // Use addRequirements() here to declare subsystem dependencies.
    lowCon = mLowCon;
    addRequirements(mLowCon);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lowCon.LowConMotorSet(0);
    lowCon.PooperMotorSet(0);
  }
}
