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
  public double driveTrainSpeed;
  public double DriveTrainTurn;
  public static double boost = Constants.BoostInactive;
  public DriveTrain() {
    leftFront = new CANSparkMax(Constants.LeftFront,MotorType.kBrushless);
    leftFront.setInverted(false);
    leftBack = new CANSparkMax(Constants.LeftBack,MotorType.kBrushless);
    leftBack.setInverted(false);
    rightFront = new CANSparkMax(Constants.RightFront,MotorType.kBrushless);
    rightFront.setInverted(true);
    rightBack = new CANSparkMax(Constants.RightBack,MotorType.kBrushless);
    rightBack.setInverted(true);

    leftBack.follow(leftFront);
    rightBack.follow(rightFront);
    drive = new DifferentialDrive(leftFront,rightFront);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void driveWithJoysticks(XboxController Controller, double speed){
    if (Constants.DeadZone * -1 < Controller.getRawAxis(Constants.XboxLeftYAxis) * -1 && Controller.getRawAxis(Constants.XboxLeftYAxis)* -1 < Constants.DeadZone) {
      driveTrainSpeed = 0;
    } else{
      if (Controller.getRawAxis(Constants.XboxLeftYAxis) * -1 > Constants.DeadZone) {
        driveTrainSpeed = boost*(Constants.DriveTrainCurve*(1-Constants.BaseVelocity)*Controller.getRawAxis(Constants.XboxLeftYAxis)*-1+(1-Constants.DriveTrainCurve)*(1-Constants.BaseVelocity)*Math.pow(Controller.getRawAxis(Constants.XboxLeftYAxis)*-1, 5))+Constants.BaseVelocity;
      }
      if (Controller.getRawAxis(Constants.XboxLeftYAxis)*-1 < Constants.DeadZone*-1) {
        driveTrainSpeed = boost*(Constants.DriveTrainCurve*(1-Constants.BaseVelocity)*Controller.getRawAxis(Constants.XboxLeftYAxis)*-1+(1-Constants.DriveTrainCurve)*(1-Constants.BaseVelocity)*Math.pow(Controller.getRawAxis(Constants.XboxLeftYAxis)*-1, 5))-Constants.BaseVelocity;
      }
      }
      if (Constants.DeadZone * -1 < Controller.getRawAxis(Constants.XboxRightXAxis) && Controller.getRawAxis(Constants.XboxRightXAxis) < Constants.DeadZone) {
        DriveTrainTurn = 0;
      } else{
        if (Controller.getRawAxis(Constants.XboxRightXAxis) > Constants.DeadZone) {
          DriveTrainTurn = boost*(Constants.DriveTrainCurve*(1-Constants.BaseVelocity)*Controller.getRawAxis(Constants.XboxRightXAxis)+(1-Constants.DriveTrainCurve)*(1-Constants.BaseVelocity)*Math.pow(Controller.getRawAxis(Constants.XboxRightXAxis), 5))+Constants.BaseVelocity;
        }
        if (Controller.getRawAxis(Constants.XboxRightXAxis) < Constants.DeadZone*-1) {
          DriveTrainTurn = boost*(Constants.DriveTrainCurve*(1-Constants.BaseVelocity)*Controller.getRawAxis(Constants.XboxRightXAxis)+(1-Constants.DriveTrainCurve)*(1-Constants.BaseVelocity)*Math.pow(Controller.getRawAxis(Constants.XboxRightXAxis), 5))-Constants.BaseVelocity;
        }
        }
    drive.arcadeDrive(driveTrainSpeed, DriveTrainTurn);
    //drive.arcadeDrive(xSpeed, zRotation);
  }
  public void driveForward(double speed){
    drive.tankDrive(speed,speed);
  }
  public void stop() {
    drive.stopMotor();
  }
}
