// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
  CANSparkMax leftFront;
  CANSparkMax rightFront;
  CANSparkMax leftBack;
  CANSparkMax rightBack;
  DifferentialDrive drive;
  public double boost = Constants.DriveTrain.BoostInactive;

  public DriveTrain() {
    leftFront = new CANSparkMax(Constants.DriveTrain.LeftFront, MotorType.kBrushless);
    leftFront.setInverted(false);
    leftBack = new CANSparkMax(Constants.DriveTrain.LeftBack, MotorType.kBrushless);
    rightFront = new CANSparkMax(Constants.DriveTrain.RightFront, MotorType.kBrushless);
    rightFront.setInverted(true);
    rightBack = new CANSparkMax(Constants.DriveTrain.RightBack, MotorType.kBrushless);

    leftBack.follow(leftFront);
    rightBack.follow(rightFront);
    drive = new DifferentialDrive(leftFront, rightFront);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void driveWithJoysticks(double throttle, double turn) {
    arcadeDrive(throttle, turn);
    // drive.arcadeDrive(xSpeed, zRotation);
  }

  public void driveForward(double speed) {
    drive.tankDrive(speed, speed);
  }

  public void arcadeDrive(double throttle, double turn) {
    drive.arcadeDrive(throttle, turn);
  }

  public void stop() {
    drive.stopMotor();
  }
}
