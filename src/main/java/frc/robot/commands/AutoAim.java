// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;

public class AutoAim extends CommandBase {
  private final DriveTrain mDriveTrain;
  private final Vision mVision;

  // Constants
  private double mSteeringKp = 0.015;
  private double minCommand = 0.3;
  private double turnCommand;
  // Network Table Entries
  NetworkTableEntry mKpSteer, mMinTa, mDrive_Kp;
  // timer
  Timer timer = new Timer();
  double time;

  /** Creates a new LimelightAim. */
  public AutoAim(DriveTrain dt, Vision v, double timeToAim) {
    // Use addRequirements() here to declare subsystem dependencies.
    mDriveTrain = dt;
    mVision = v;
    addRequirements(dt);
    time = timeToAim;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double throttle = RobotContainer.driverJoyStick.getLeftY();
    if (mVision.isTargetValid()) {
      double mTx = mVision.getTx();
      if (Math.abs(mTx) > 1) {
        turnCommand = mSteeringKp * mTx + Math.copySign(minCommand, mTx);
      } else {
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
    return (timer.get() > time);
  }
}
