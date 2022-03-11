// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.ShuffleboardInfo;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;

public class DriveWithLimelight extends CommandBase {
  private final DriveTrain mDriveTrain;
  private final Vision mVision;

  // Constants
  private double mSteeringKp = 0.05;
  private double mDriveKp = 0.80;
  private double steeringAdjust;
  private double mTx;
  private double headingError;
  private double minCommand = 0.0;
  private double turnCommand;
  // Network Table Entries
  NetworkTableEntry mKpSteer, mMinTa, mDrive_Kp;

  /** Creates a new LimelightAim. */
  public DriveWithLimelight(DriveTrain dt, Vision v) {
    // Use addRequirements() here to declare subsystem dependencies.
    mDriveTrain = dt;
    mVision = v;
    addRequirements(dt, v);


  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double throttle = RobotContainer.driverJoyStick.getLeftY();
    if (mVision.isTargetValid()) {
      mTx = mVision.getTx();
      headingError = -mTx;
      if (mTx < -0.5) {
        turnCommand = mSteeringKp * headingError; //- minCommand;
      } else if (mTx > 0.5) {
        turnCommand = mSteeringKp * headingError; //+ minCommand;
      } else if (mTx < 0.5 && mTx > -0.5) {
        turnCommand = 0;
      }
      mDriveTrain.arcadeDrive(throttle, turnCommand);
    }
    else mDriveTrain.arcadeDrive(throttle, -RobotContainer.driverJoyStick.getRightX());
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
