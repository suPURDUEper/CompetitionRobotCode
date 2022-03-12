// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LowerConveyor;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.UpperConveyor;

public class Purge extends CommandBase {
  /** Creates a new Purge. */
  private final LowerConveyor lowCon;
  private final Intake intake;
  private final UpperConveyor upperCon;
  private final Shooter shooter;
  public Purge(Intake mIntake, LowerConveyor mLowCon, UpperConveyor mUpperCon, Shooter mShooter) {
    intake = mIntake;
    lowCon = mLowCon;
    upperCon = mUpperCon;
    shooter = mShooter;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setAcceleratorTargetRPM(-6300);
    upperCon.ConveyorMotorSet(-0.5);
    lowCon.PooperMotorSet(-0.3);
    lowCon.LowConMotorSet(-1);
    intake.IndexerMotorSet(-1);
    intake.IntakeMotorSet(-1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setAcceleratorTargetRPM(-6300);
    upperCon.ConveyorMotorSet(0);
    lowCon.PooperMotorSet(0);
    lowCon.LowConMotorSet(0);
    intake.IndexerMotorSet(0);
    intake.IntakeMotorSet(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
