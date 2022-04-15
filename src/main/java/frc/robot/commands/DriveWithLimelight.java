// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.ShuffleboardInfo;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;

public class DriveWithLimelight extends CommandBase {
  private final DriveTrain mDriveTrain;
  private final Vision mVision;

  // Constants
  private double mSteeringKp = 0.015;
  private double minCommand = 0.3;
  private double turnCommand;
  private Supplier<Double> throttle;
  // Network Table Entries
  NetworkTableEntry mKpSteer, mMinTa, mDrive_Kp;

  /** Creates a new LimelightAim. */

  public DriveWithLimelight(DriveTrain dt, Vision v) {
    this(dt, v, () -> 0.0);
  }

  public DriveWithLimelight(DriveTrain dt, Vision v, Supplier<Double> throttle) {
    // Use addRequirements() here to declare subsystem dependencies.
    mDriveTrain = dt;
    mVision = v;
    addRequirements(dt);
    this.throttle = throttle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (mVision.isTargetValid()) {
      double mTx = mVision.getTx();
      if (Math.abs(mTx) > 1) {
        turnCommand = mSteeringKp * mTx + Math.copySign(minCommand, mTx);
      } else {
        turnCommand = 0;
      }
      mDriveTrain.arcadeDrive(-1 * throttle.get(), turnCommand);
    }
    else mDriveTrain.arcadeDrive(-1 * throttle.get(), RobotContainer.driverJoyStick.getRightX());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
