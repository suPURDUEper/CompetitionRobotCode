package frc.robot.subsystems;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  private TalonFX masterClimbMotor;
  private TalonFX followerClimbMotor;
  /**
   * Creates a new climber object.
   */
  public Climber() {
    // double acting solenoid
    // double acting cylinder

    masterClimbMotor = new TalonFX(Constants.leftClimbMotor);
    masterClimbMotor.setInverted(false);

    followerClimbMotor = new TalonFX(Constants.rightClimbMotor);
    // any command give to the master climb motor is now also passed to the follower
    followerClimbMotor.follow(followerClimbMotor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * 
   * @param controller the controller object for the operator
   */
  public void climb(XboxController controller) {
    // getRightY() is a new function introduced in the 2022 wpilib
    // removes the need for axis id
    masterClimbMotor.set(ControlMode.PercentOutput, controller.getRightY());
  }
}
