package frc.robot.subsystems;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  private final DoubleSolenoid climberSolenoid;
  private final WPI_TalonFX leftClimbMotor;
  private final WPI_TalonFX rightClimbMotor;
  /**
   * Creates a new climber object.
   */
  public Climber() {
    // double acting solenoid
    // double acting cylinder

    leftClimbMotor = new WPI_TalonFX(Constants.Climber.LEFT_CLIMB_MOTOR_CAD_ID);
    setupTalon(leftClimbMotor);
    leftClimbMotor.setInverted(false);
    leftClimbMotor.selectProfileSlot(0, 0);
		leftClimbMotor.config_kF(0, Constants.Climber.LEFT_CLIMB_MOTOR_KF, Constants.TALON_TIMEOUT);
		leftClimbMotor.config_kP(0, Constants.Climber.LEFT_CLIMB_MOTOR_KP, Constants.TALON_TIMEOUT);
		leftClimbMotor.config_kI(0, Constants.Climber.LEFT_CLIMB_MOTOR_KI, Constants.TALON_TIMEOUT);
		leftClimbMotor.config_kD(0, Constants.Climber.LEFT_CLIMB_MOTOR_KD, Constants.TALON_TIMEOUT);
    leftClimbMotor.configMotionCruiseVelocity(Constants.Climber.CLIMB_MAX_VELOCITY, Constants.TALON_TIMEOUT);
		leftClimbMotor.configMotionAcceleration(Constants.Climber.CLIMB_MAX_ACCELERATION, Constants.TALON_TIMEOUT);

    rightClimbMotor = new WPI_TalonFX(Constants.Climber.LEFT_CLIMB_MOTOR_CAD_ID);
    setupTalon(rightClimbMotor);
    rightClimbMotor.setInverted(false);
    rightClimbMotor.selectProfileSlot(0, 0);
		rightClimbMotor.config_kF(0, Constants.Climber.LEFT_CLIMB_MOTOR_KF, Constants.TALON_TIMEOUT);
		rightClimbMotor.config_kP(0, Constants.Climber.LEFT_CLIMB_MOTOR_KP, Constants.TALON_TIMEOUT);
		rightClimbMotor.config_kI(0, Constants.Climber.LEFT_CLIMB_MOTOR_KI, Constants.TALON_TIMEOUT);
		rightClimbMotor.config_kD(0, Constants.Climber.LEFT_CLIMB_MOTOR_KD, Constants.TALON_TIMEOUT);
    rightClimbMotor.configMotionCruiseVelocity(Constants.Climber.CLIMB_MAX_VELOCITY, Constants.TALON_TIMEOUT);
		rightClimbMotor.configMotionAcceleration(Constants.Climber.CLIMB_MAX_ACCELERATION, Constants.TALON_TIMEOUT);


    climberSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 4, 5);
    // initialize the climber to forward
    // this way the toggle function works
    climberSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Climb Motor Position", leftClimbMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Right Climb Motor Position", rightClimbMotor.getSelectedSensorPosition());
    
  }
  public void climberUp() {
    leftClimbMotor.set(ControlMode.MotionMagic, Constants.Climber.LEFT_CLIMB_EXTEND_HEIGHT);
    rightClimbMotor.set(ControlMode.MotionMagic, Constants.Climber.RIGHT_CLIMB_EXTEND_HEIGHT);


  } 
  public void climberDown() {
    leftClimbMotor.set(ControlMode.MotionMagic, 0);
    rightClimbMotor.set(ControlMode.MotionMagic, 0);
  }
  /**
   * 
   * @param controller the controller object for the operator
   */
  public void climb(XboxController controller) {
    // getRightY() is a new function introduced in the 2022 wpilib
    // removes the need for axis id
    leftClimbMotor.set(ControlMode.PercentOutput, controller.getLeftY());
    rightClimbMotor.set(ControlMode.PercentOutput, controller.getLeftY());
  }
  public void extendSolenoid() {
      climberSolenoid.set(Value.kForward);
  }
  public void retractSolenoid() {
    climberSolenoid.set(Value.kReverse);
  }

  private void setupTalon(WPI_TalonFX talonFX) {
    talonFX.configFactoryDefault();
    talonFX.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.TALON_TIMEOUT);
    talonFX.configNeutralDeadband(0.001, Constants.TALON_TIMEOUT);
    talonFX.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.TALON_TIMEOUT);
		talonFX.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.TALON_TIMEOUT);
    talonFX.configNominalOutputForward(0, Constants.TALON_TIMEOUT);
		talonFX.configNominalOutputReverse(0, Constants.TALON_TIMEOUT);
		talonFX.configPeakOutputForward(1, Constants.TALON_TIMEOUT);
		talonFX.configPeakOutputReverse(-1, Constants.TALON_TIMEOUT);
    talonFX.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 60, 40, 0.5));
  }


}
