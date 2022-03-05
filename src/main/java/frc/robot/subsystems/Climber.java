package frc.robot.subsystems;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import static frc.robot.Constants.Climber.*;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StickyFaults;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.utils.TalonUtils;

public class Climber extends PIDSubsystem {
  private final DoubleSolenoid climberSolenoid;
  private final WPI_TalonFX leftClimbMotor;
  private final WPI_TalonFX rightClimbMotor;
  private double climbSyncCorrection;
  /**
   * Creates a new climber object.
   */
  public Climber() {
    super(new PIDController(CLIMB_SYNC_KP, CLIMB_SYNC_KI, CLIMB_SYNC_KD), 0);
    leftClimbMotor = new WPI_TalonFX(LEFT_CLIMB_MOTOR_CAD_ID);
    leftClimbMotor.setInverted(false);
    leftClimbMotor.configAllSettings(getClimberTalonConfig());
    leftClimbMotor.setNeutralMode(NeutralMode.Brake);
    leftClimbMotor.setSelectedSensorPosition(0);

    rightClimbMotor = new WPI_TalonFX(RIGHT_CLIMB_MOTOR_CAN_ID);
    rightClimbMotor.setInverted(false);
    rightClimbMotor.configAllSettings(getClimberTalonConfig());
    rightClimbMotor.setNeutralMode(NeutralMode.Brake);
    rightClimbMotor.setSelectedSensorPosition(0);

    climberSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 4, 5);
    // initialize the climber to forward
    // this way the toggle function works
    climberSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    super.periodic();
    SmartDashboard.putNumber("Left Climb Motor Position", leftClimbMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Right Climb Motor Position", rightClimbMotor.getSelectedSensorPosition());

    // Reset position if limit switch is triggered
    if (leftClimbMotor.getSensorCollection().isRevLimitSwitchClosed() == 1) {
      leftClimbMotor.setSelectedSensorPosition(0);
    }
    if (rightClimbMotor.getSensorCollection().isRevLimitSwitchClosed() == 1) {
      rightClimbMotor.setSelectedSensorPosition(0);
    }
  }

  @Override
  protected void useOutput(double output, double setpoint) {
    output = climbSyncCorrection;
  }

  @Override
  protected double getMeasurement() {
    return leftClimbMotor.getSelectedSensorPosition() - rightClimbMotor.getSelectedSensorPosition();
  }

  @Override 
  public void disable() {
    super.disable();
    climbSyncCorrection = 0;
  }

  public void setPower(double power) {
    // Cap at 95% speed so we have some room to sync
    if (Math.abs(power) > 0.95) {
      power = Math.copySign(0.95, power);
    }
    // Left higher than right -> difference is positive -> error is negative -> correction is negative.
    // Left is higher means we need to slow down left, so add correction to left (and subtract from right)
    leftClimbMotor.set(ControlMode.PercentOutput, power + climbSyncCorrection);
    rightClimbMotor.set(ControlMode.PercentOutput, power - climbSyncCorrection);
  }

  public void climberTilt() {
    climberSolenoid.set(Value.kForward);
  }

  public void climberStraight() {
    climberSolenoid.set(Value.kReverse);
  }

  public boolean isClimberUp() {
    boolean leftAtTop, rightAtTop;
    StickyFaults stickyFaults = new StickyFaults();
    leftClimbMotor.getStickyFaults(stickyFaults);
    leftAtTop = stickyFaults.ForwardSoftLimit;
    rightClimbMotor.getStickyFaults(stickyFaults);
    rightAtTop = stickyFaults.ForwardSoftLimit;
    return leftAtTop && rightAtTop;
  }

  public boolean isClimberDown() {
    boolean leftDown = leftClimbMotor.getSensorCollection().isRevLimitSwitchClosed() == 1;
    boolean rightDown = rightClimbMotor.getSensorCollection().isRevLimitSwitchClosed() == 1;
    return leftDown && rightDown;
  }

  public void clearStickyFaults() {
    leftClimbMotor.clearStickyFaults();
    rightClimbMotor.clearStickyFaults();
  }

  private TalonFXConfiguration getClimberTalonConfig() {
    TalonFXConfiguration config = TalonUtils.getDefaultTalonConfig();
    config.supplyCurrLimit = new SupplyCurrentLimitConfiguration(true, 60, 40, 0.5);
    config.forwardSoftLimitThreshold = CLIMB_MAX_HEIGHT;
    config.forwardSoftLimitEnable = true;
    config.reverseLimitSwitchNormal = LimitSwitchNormal.NormallyOpen;
    config.reverseLimitSwitchSource = LimitSwitchSource.FeedbackConnector;
    return config;
  }
}
