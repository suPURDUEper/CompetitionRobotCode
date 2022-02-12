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
  public static double boost = Constants.DriveTrain.BoostInactive;

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

  public void driveWithJoysticks(XboxController Controller, double speed) {
    double LeftYAxis = Controller.getRawAxis(Constants.Controller.XboxLeftYAxis) * -1;
    double RightXAxis = Controller.getRawAxis(Constants.Controller.XboxRightXAxis);
    double driveTrainSpeed = getThrottleMap(LeftYAxis);
    double DriveTrainTurn = getTurnMap(RightXAxis);

    arcadeDrive(driveTrainSpeed, DriveTrainTurn);
    // drive.arcadeDrive(xSpeed, zRotation);
  }

  public void driveForward(double speed) {
    drive.tankDrive(speed, speed);
  }

  public void arcadeDrive(double throttle, double turn) {
    drive.arcadeDrive(throttle, turn);
  }

  public double getThrottleMap(double LeftYAxis) {
    double driveTrainSpeed = 0;
    if (Constants.Controller.DeadZone * -1 < LeftYAxis && LeftYAxis < Constants.Controller.DeadZone) {
      driveTrainSpeed = 0;
    } else {
      if (LeftYAxis > Constants.Controller.DeadZone) {
        driveTrainSpeed = boost * (Constants.DriveTrain.DriveTrainCurve * (1 - Constants.DriveTrain.BaseVelocity)
            * LeftYAxis * -1
            + (1 - Constants.DriveTrain.DriveTrainCurve)
                * (1 - Constants.DriveTrain.BaseVelocity) * Math.pow(LeftYAxis * -1, 5))
            + Constants.DriveTrain.BaseVelocity;
      }
      if (LeftYAxis < Constants.Controller.DeadZone * -1) {
        driveTrainSpeed = boost * (Constants.DriveTrain.DriveTrainCurve * (1 - Constants.DriveTrain.BaseVelocity)
            * LeftYAxis * -1
            + (1 - Constants.DriveTrain.DriveTrainCurve)
                * (1 - Constants.DriveTrain.BaseVelocity) * Math.pow(LeftYAxis * -1, 5))
            - Constants.DriveTrain.BaseVelocity;
      }
    }
    return driveTrainSpeed;
  }
  public double getTurnMap(double RightXAxis) {
    double DriveTrainTurn = 0;
    if (Constants.Controller.DeadZone * -1 < RightXAxis && RightXAxis < Constants.Controller.DeadZone) {
      DriveTrainTurn = 0;
    } else {
      if (RightXAxis > Constants.Controller.DeadZone) {
        DriveTrainTurn = boost * (Constants.DriveTrain.DriveTrainCurve * (1 - Constants.DriveTrain.BaseVelocity)
            * RightXAxis
            + (1 - Constants.DriveTrain.DriveTrainCurve) * (1 - Constants.DriveTrain.BaseVelocity)
                * Math.pow(RightXAxis, 5))
            + Constants.DriveTrain.BaseVelocity;
      }
      if (RightXAxis < Constants.Controller.DeadZone * -1) {
        DriveTrainTurn = boost * (Constants.DriveTrain.DriveTrainCurve * (1 - Constants.DriveTrain.BaseVelocity)
            * RightXAxis)
            + (1 - Constants.DriveTrain.DriveTrainCurve)
                * (1 - Constants.DriveTrain.BaseVelocity) * Math.pow(RightXAxis, 5)
            - Constants.DriveTrain.BaseVelocity;
      }
    }
    return DriveTrainTurn;
  }


  public void stop() {
    drive.stopMotor();
  }
}
