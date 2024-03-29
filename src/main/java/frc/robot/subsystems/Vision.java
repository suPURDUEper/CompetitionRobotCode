// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  private final NetworkTable mLimelightTable;
  private double tv, tx, ty;
  private boolean mIsTargetValid;
  private final NetworkTableEntry mLedEntry, mTv;
  /** Creates a new Vision. */
  public Vision() {
    mTv = Shuffleboard.getTab("Limelight Aim").add("Tv", 0.0).getEntry();
    mLimelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    mLedEntry = mLimelightTable.getEntry("ledmode");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    tv = mLimelightTable.getEntry("tv").getDouble(0.0);
    tx = mLimelightTable.getEntry("tx").getDouble(0.0);
    ty = mLimelightTable.getEntry("ty").getDouble(0.0);
    mIsTargetValid = isTargetValid();
    mTv.forceSetDouble(tv);
  }

  public double getTx() {
    return tx;
  }

  public double getTy() {
    return ty;
  }

  public boolean isTargetValid() {
    return (tv == 1.0);
  }

  public void setL1LedMode(int mode) {
    mLedEntry.forceSetNumber(mode);
  }


}
