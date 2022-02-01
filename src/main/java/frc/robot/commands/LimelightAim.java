// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ShuffleboardInfo;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;

public class LimelightAim extends CommandBase {
  private final DriveTrain mDriveTrain;
  private final Vision mVision;

  // Constants
  private double mSteeringKp = 0.05;
  private double mDriveKp = 0.80;
  private double steeringAdjust = 0.0;
  private double mTx = 0.0;
  private double headingError = 0.0;
  private double minCommand = 0.05;
  private double leftCommand, rightCommand = 0.0;
  // Network Table Entries
  NetworkTableEntry mKpSteer, mMinTa, mDrive_Kp;

  /** Creates a new LimelightAim. */
  public LimelightAim(DriveTrain dt, Vision v) {
    // Use addRequirements() here to declare subsystem dependencies.
    mDriveTrain = dt;
    mVision = v;
    addRequirements(dt, v);

    mKpSteer = ShuffleboardInfo.getInstance().getKpSteer();
    mDrive_Kp = ShuffleboardInfo.getInstance().getKpDrive();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mSteeringKp = mKpSteer.getDouble(0);
    mDriveKp = mDrive_Kp.getDouble(0);
    steeringAdjust = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (mVision.isTargetValid()) {
      mTx = mVision.getTx();
      headingError = -mTx;
      if (mTx > 1.0) {
        steeringAdjust = mSteeringKp * headingError - minCommand;
      } else if (mTx < 1.0) {
        steeringAdjust = mSteeringKp * headingError + minCommand;
      }
      leftCommand += steeringAdjust;
      rightCommand -= steeringAdjust;

      mDriveTrain.tankDrive(leftCommand, rightCommand);
    }
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
