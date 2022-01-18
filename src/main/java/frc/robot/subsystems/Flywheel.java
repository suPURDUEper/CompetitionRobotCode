// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Flywheel extends SubsystemBase {
  TalonFX leftFlywheel;
  TalonFX rightFlywheel;
  public Flywheel() {
    leftFlywheel = new TalonFX(Constants.LeftFlywheel);
    leftFlywheel.setInverted(false);
    rightFlywheel = new TalonFX(Constants.RightFlywheel);
    rightFlywheel.setInverted(true);
    rightFlywheel.follow(leftFlywheel);

    leftFlywheel.config_kF(Constants.kPIDLoopIdx, Constants.FlywheelkF, Constants.kTimeoutMs);
		leftFlywheel.config_kP(Constants.kPIDLoopIdx, Constants.FlywheelkP, Constants.kTimeoutMs);
		leftFlywheel.config_kI(Constants.kPIDLoopIdx, Constants.FlywheelkI, Constants.kTimeoutMs);
		leftFlywheel.config_kD(Constants.kPIDLoopIdx, Constants.FlywheelkD, Constants.kTimeoutMs);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void SetSpeed(double speed) {
    leftFlywheel.set(ControlMode.Velocity, speed * 2048 / 600);
  }
  
}
