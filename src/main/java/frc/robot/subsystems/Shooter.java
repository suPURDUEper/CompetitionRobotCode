// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.Shooter.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ShuffleboardInfo;
import frc.robot.util.LookupTable;
import frc.robot.util.TalonUtils;


/**
 * 2022's shooter uses a 2 position hood with a variable velovity flywheel.
 * 
 * This takes care of both the Fender shot, the close up against the wall option, and the
 * Distance shot, which will be anything further than the Tarmac but not further than 15 feet
 */
public class Shooter extends SubsystemBase {
  // Servo class
  private final DoubleSolenoid hood;
  // Flywheel Falcon
  private final WPI_TalonFX leftFlywheelMotor;
  private final WPI_TalonFX rightFlywheelMotor;
  private final WPI_TalonFX acceleratorWheelMotor;

  // Simulator
  private FlywheelSim flywheelSim;
  private TalonFXSimCollection flywheelLeftMotorSim;
  private TalonFXSimCollection flywheelRightMotorSim;

  // Target RPM
  private int targetFlywheelRpm = 0;
  private int targetAcceleratorRpm = 0;
  NetworkTableEntry flywheNetworkTableEntry;
  
  boolean isShooterEnabled = false;
  double atSpeedTimer;

  LookupTable lookupTable;
  
  /**
   * Create a new shooter subsystem
   */
  public Shooter() {
    // Hood defines
    hood = new DoubleSolenoid(PneumaticsModuleType.REVPH, HOOD_SOLENOID_FWD_ID, HOOD_SOLENOID_REV_ID);
    
    // Motor defines
    leftFlywheelMotor = new WPI_TalonFX(LEFT_FLYWHEEL_CAN_ID);
    rightFlywheelMotor = new WPI_TalonFX(RIGHT_FLYWHEEL_CAN_ID);
    acceleratorWheelMotor = new WPI_TalonFX(ACCELERATOR_CAN_ID);

    //Setup left shooter motor
    reinitTalonFx(leftFlywheelMotor);
    leftFlywheelMotor.setInverted(false);
    setTalonFXPidGains(leftFlywheelMotor);

    //Setup right shooter motor
    reinitTalonFx(rightFlywheelMotor);
    rightFlywheelMotor.setInverted(true);
    rightFlywheelMotor.follow(leftFlywheelMotor);
    TalonUtils.setStatusFramePeriodFollower(rightFlywheelMotor);

    //Setup accelerator wheel
    reinitTalonFx(acceleratorWheelMotor);
    acceleratorWheelMotor.setInverted(false);
    setTalonFXPidGains(acceleratorWheelMotor);

    // Simulate
    flywheelSim = new FlywheelSim(DCMotor.getFalcon500(2), 1.0, 0.001463);
    flywheelLeftMotorSim = leftFlywheelMotor.getSimCollection();
    flywheelRightMotorSim = rightFlywheelMotor.getSimCollection();

    flywheNetworkTableEntry = ShuffleboardInfo.getInstance().getFlywheelSpeed();

    lookupTable = new LookupTable();
    lookupTable.addValue(22.0, 2150);
    lookupTable.addValue(17.1, 2250);
    lookupTable.addValue(10.9, 2350);
    lookupTable.addValue(4.9, 2450);
    lookupTable.addValue(-0.9, 2650);
    lookupTable.addValue(-1.33, 2700);
    lookupTable.addValue(-4.4, 2850);
    lookupTable.addValue(-4.75, 2950);

  }

  /**
   * Meant for right up against the fender
   */
  public void setFenderHoodPosition() {
    hood.set(Value.kForward);
  }

  /**
   * Meant for anything further than the Tarmac
   */
  public void setDistanceHoodPosition() {
    hood.set(Value.kReverse);
  }

  /**
   * Set the flywheel speed
   * @param speed taken in units of RPM
   */
  public void setFlywheelTargetRPM(int rpm) {
    targetFlywheelRpm = rpm;
    flywheNetworkTableEntry.setNumber(rpm);
  }

    /**
   * Set the flywheel speed based on limelight
   * @param speed taken in units of RPM
   */
  public void setFlywheelDistanceRPM(double tx) {
    int rpm = (int) lookupTable.getValue(tx);
    targetFlywheelRpm = rpm;
    flywheNetworkTableEntry.setNumber(rpm);
  }

  /**
   * Set the flywheel speed based on limelight
   * @param speed taken in units of RPM
   */
  public void setAcceleratorDistanceRPM(double tx) {
    int rpm = (int) lookupTable.getValue(tx);
    targetAcceleratorRpm = rpm;
  }

  /**
   * @return the speed of the motor in units of RPM
   */
  public double getFlywheelRPM() {
    return talonFXUnitsToRpm(leftFlywheelMotor.getSelectedSensorVelocity());
  }

  public void enableShooter() {
    isShooterEnabled = true;
  }

  public void disableShooter() {
    isShooterEnabled = false;
  }

  /**
   * Set the accelerator wheel speed
   * @param rpm taken in units of RPM
   */
  public void setAcceleratorTargetRPM(int rpm) {
    targetAcceleratorRpm = rpm;
  }

  /**
   * @return the speed of the motor in units of RPM
   */
  public double getAcceleratorRPM() {
    return talonFXUnitsToRpm(acceleratorWheelMotor.getClosedLoopTarget());
  }

  @Override
  public void simulationPeriodic() {
    double vBat = BatterySim.calculateDefaultBatteryLoadedVoltage(leftFlywheelMotor.getSupplyCurrent(), rightFlywheelMotor.getSupplyCurrent());
    flywheelLeftMotorSim.setBusVoltage(vBat);
    SmartDashboard.putNumber("Battery Voltage", vBat);
    SmartDashboard.putNumber("Flywheel Target RPM", talonFXUnitsToRpm(leftFlywheelMotor.getClosedLoopTarget()));
    SmartDashboard.putNumber("Flywheel Motor Error", talonFXUnitsToRpm(leftFlywheelMotor.getClosedLoopError()));
    SmartDashboard.putNumber("Flywheel Motor Current", rightFlywheelMotor.getSupplyCurrent());
    double vMotor = flywheelLeftMotorSim.getMotorOutputLeadVoltage();
    flywheelSim.setInputVoltage(vMotor);
    SmartDashboard.putNumber("Flywheel Motor Voltage", vMotor);
    flywheelSim.update(0.02);
    flywheelLeftMotorSim.setIntegratedSensorVelocity((int) rpmToTalonFXUnits(flywheelSim.getAngularVelocityRPM()));
    flywheelLeftMotorSim.setSupplyCurrent(flywheelSim.getCurrentDrawAmps() * -1);
    flywheelRightMotorSim.setSupplyCurrent(flywheelSim.getCurrentDrawAmps() * -1);
    SmartDashboard.putNumber("Flywheel Speed", flywheelSim.getAngularVelocityRPM());
  }

  @Override
  public void periodic() {
    targetFlywheelRpm = flywheNetworkTableEntry.getNumber(0).intValue();
    if (isShooterEnabled) {
      leftFlywheelMotor.set(ControlMode.Velocity, rpmToTalonFXUnits(targetFlywheelRpm));
      acceleratorWheelMotor.set(ControlMode.Velocity, rpmToTalonFXUnits(targetAcceleratorRpm));
    } else {
      leftFlywheelMotor.set(ControlMode.Disabled, 0);
      acceleratorWheelMotor.set(ControlMode.Disabled, 0);
      }
      SmartDashboard.putNumber("Flywheel Speed", talonFXUnitsToRpm(leftFlywheelMotor.getSelectedSensorVelocity()));
      SmartDashboard.putNumber("Accelerator Speed", talonFXUnitsToRpm(acceleratorWheelMotor.getSelectedSensorVelocity()));

  }

  private double rpmToTalonFXUnits(double rpm) {
    return rpm * 2048 / 600;
  }

  private double talonFXUnitsToRpm(double talonFXUnit) {
    return (talonFXUnit / 2048) * 10 * 60;
  }

  public boolean isShooterAtSpeed() {
    if (Math.abs(getFlywheelRPM() - targetFlywheelRpm) < SHOOTER_RPM_TOLERANCE) {
      // Must be at the target RPM for a certain amount of loops in a row before saying
      // it's safe to fire. 
      return RobotController.getFPGATime() > (atSpeedTimer + SHOOTER_RPM_STABLE_TIME);
    } else {
      atSpeedTimer = RobotController.getFPGATime();
      return false;
    }
  }

  private void reinitTalonFx(TalonFX talonFX) {
    talonFX.configFactoryDefault();
    talonFX.configNeutralDeadband(0.001);
    talonFX.setNeutralMode(NeutralMode.Coast);
    talonFX.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 
      SHOOTER_CURRENT_LIMIT_AMPS, SHOOTER_CURRENT_LIMIT_AMPS + 5, 1));
    talonFX.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, PID_LOOP_INDEX, 30);
    talonFX.configNominalOutputReverse(0, 30);
    talonFX.configPeakOutputForward(1, 30);
    talonFX.configPeakOutputReverse(-1, 30);
  }

  private void setTalonFXPidGains(TalonFX talonFX) {
    talonFX.config_kF(PID_LOOP_INDEX, FLYWHEEL_KF, 30);
    talonFX.config_kP(PID_LOOP_INDEX, FLYWHEEL_KP, 30);
    talonFX.config_kI(PID_LOOP_INDEX, FLYWHEEL_KI, 30);
    talonFX.config_kD(PID_LOOP_INDEX, FLYWHEEL_KD, 30);
  }
}
