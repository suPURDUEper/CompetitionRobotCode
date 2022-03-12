// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LowerConveyor;
import frc.robot.subsystems.UpperConveyor;

public class ShootBall extends CommandBase {
  private final UpperConveyor upperConveyor;
  private final LowerConveyor lowerConveyor;
  private Supplier<Boolean> safeToFire;

  /** Creates a new ShootBall. */
  public ShootBall(UpperConveyor mUpperCon, LowerConveyor mLowCon, Supplier<Boolean> safeToFire) {
    // Use addRequirements() here to declare subsystem dependencies.
    upperConveyor = mUpperCon;
    lowerConveyor = mLowCon;
    this.safeToFire = safeToFire;
    addRequirements(mUpperCon, mLowCon);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (safeToFire.get()) {
      lowerConveyor.setLowerConveyorPercentOutput(0.7);
      lowerConveyor.setPooperPercentOutput(0.7);
      upperConveyor.setPercentOutput(1);
    } else {
      lowerConveyor.setLowerConveyorPercentOutput(0);
      lowerConveyor.setPooperPercentOutput(0);
      upperConveyor.setPercentOutput(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    lowerConveyor.setLowerConveyorPercentOutput(0);
    lowerConveyor.setPooperPercentOutput(0);
    upperConveyor.setPercentOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
