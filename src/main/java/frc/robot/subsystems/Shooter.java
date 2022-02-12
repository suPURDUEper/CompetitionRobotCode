// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * 2022's shooter uses a 2 position hood with a variable velovity flywheel.
 * 
 * This takes care of both the Fender shot, the close up against the wall option, and the
 * Distance shot, which will be anything further than the Tarmac but not further than 15 feet
 */
public class Shooter extends SubsystemBase {
  // Servo maxes to control hood position
  private final double max = 1.75;
  private final double  deadbandMax = 1.7;
  private final double center = 1.5;
  private final double deadbandMin = 1.2;
  private final double min = 1.1;
  // Servo class
  private final Servo hoodServo;
  // Flywheel Falcon
  private final TalonFX leftFlywheelMotor;
  private final TalonFX rightFlywheelMotor;

  /**
   * Create a new shooter subsystem
   */
  public Shooter() {
    // Hood Servo defines
    hoodServo = new Servo(Constants.Shooter.HOOD_SERVO_CHANNEL);
    hoodServo.setBounds(max, deadbandMax, center, deadbandMin, min);
    // Motor defines
    leftFlywheelMotor = new TalonFX(Constants.Shooter.LeftFlywheel);
    leftFlywheelMotor.setInverted(false);
    rightFlywheelMotor = new TalonFX(Constants.Shooter.RightFlywheel);
    // this is opposite the left flywheel so we need it to be inverted to go the correct direction
    rightFlywheelMotor.setInverted(true);
    rightFlywheelMotor.follow(leftFlywheelMotor);
    // PID setting for the flywheel, this gets applied to both motors since the right follow the left
    leftFlywheelMotor.config_kF(Constants.Shooter.kPIDLoopIdx, Constants.Shooter.FlywheelkF, Constants.Shooter.kTimeoutMs);
		leftFlywheelMotor.config_kP(Constants.Shooter.kPIDLoopIdx, Constants.Shooter.FlywheelkP, Constants.Shooter.kTimeoutMs);
		leftFlywheelMotor.config_kI(Constants.Shooter.kPIDLoopIdx, Constants.Shooter.FlywheelkI, Constants.Shooter.kTimeoutMs);
		leftFlywheelMotor.config_kD(Constants.Shooter.kPIDLoopIdx, Constants.Shooter.FlywheelkD, Constants.Shooter.kTimeoutMs);
  }

  /**
   * Meant for right up against the fender
   */
  public void setFenderHoodPosition() {
    // This is unextended
    hoodServo.setSpeed(0.1);
  }

  /**
   * Meant for anything further than the Tarmac
   */
  public void setDistanceHoodPosition() {
    // This is fully extended
    hoodServo.setSpeed(0.9);
  }

  /**
   * Mainly for testing purposes
   * Cna be set via the Smart Dashboard
   * @param position between 0 - 1
   */
  public void setVariableHoodPosition(double position) {
    hoodServo.setSpeed(position);
  }

  /**
   * Set the flywheel speed
   * @param speed taken in units of RPM
   */
  public void setSpeed(double speed) {
    leftFlywheelMotor.set(ControlMode.Velocity, speed * 2048 / 600);
  }

  /**
   * NOT YET IMPLEMENTED
   * @return the speed of the motor in units of RPM
   */
  public double getSpeed() {
    // This is to be filled in later...
    return 0.0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
