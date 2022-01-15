// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {
  /** Creates a new Indexer. */
  CANSparkMax indexerMoter1;
  CANSparkMax indexerMotor2;
  CANSparkMax indexerMotor3;
  public Indexer() {
    indexerMoter1 = new CANSparkMax(Constants.IndexerMotor1,MotorType.kBrushless);
    indexerMoter1.setInverted(false);
    indexerMotor2 = new CANSparkMax(Constants.IndexerMotor2,MotorType.kBrushless);
    indexerMotor2.setInverted(false);
    indexerMotor3 = new CANSparkMax(Constants.IndexerMotor3,MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
