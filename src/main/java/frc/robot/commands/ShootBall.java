// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LowerConveyor;
import frc.robot.subsystems.UpperConveyor;

public class ShootBall extends CommandBase {
  private final UpperConveyor upperCon;
  private final LowerConveyor lowCon;
  /** Creates a new ShootBall. */
  public ShootBall(UpperConveyor mUpperCon, LowerConveyor mLowCon) {
    // Use addRequirements() here to declare subsystem dependencies.
    upperCon = mUpperCon;
    lowCon = mLowCon;
    addRequirements(mUpperCon, mLowCon);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    lowCon.LowConMotorSet(0.7);
    lowCon.PooperMotorSet(0.7);
    upperCon.ConveyorMotorSet(1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    lowCon.LowConMotorSet(0);
    lowCon.PooperMotorSet(0);
    upperCon.ConveyorMotorSet(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
