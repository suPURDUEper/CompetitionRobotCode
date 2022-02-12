// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


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
  
  /**
   * Create a new shooter subsystem
   */
  public Shooter() {
    // Hood defines
    hood = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.IDs.HOOD_SOLENOID_FWD_ID, Constants.IDs.HOOD_SOLENOID_REV_ID);
    
    // Motor defines
    leftFlywheelMotor = new WPI_TalonFX(Constants.Shooter.LEFT_FLYWHEEL_CAN_ID);
    rightFlywheelMotor = new WPI_TalonFX(Constants.Shooter.RIGHT_FLYWHEEL_CAN_ID);
    acceleratorWheelMotor = new WPI_TalonFX(Constants.Shooter.ACCELERATOR_CAN_ID);

    //Setup left shooter motor
    reinitTalonFx(leftFlywheelMotor);
    leftFlywheelMotor.setInverted(false);
    setTalonFXPidGains(leftFlywheelMotor);

    //Setup right shooter motor
    reinitTalonFx(rightFlywheelMotor);
    rightFlywheelMotor.setInverted(true);
    rightFlywheelMotor.follow(leftFlywheelMotor);

    //Setup accelerator wheel
    reinitTalonFx(acceleratorWheelMotor);
    acceleratorWheelMotor.setInverted(false);
    setTalonFXPidGains(acceleratorWheelMotor);

    // Simulate
    flywheelSim = new FlywheelSim(DCMotor.getFalcon500(2), 1.0, 0.001463);
    flywheelLeftMotorSim = leftFlywheelMotor.getSimCollection();
    flywheelRightMotorSim = rightFlywheelMotor.getSimCollection();

  }

  /**
   * Meant for right up against the fender
   */
  public void setFenderHoodPosition() {
    hood.set(Value.kReverse);
  }

  /**
   * Meant for anything further than the Tarmac
   */
  public void setDistanceHoodPosition() {
    hood.set(Value.kForward);
  }

  /**
   * Set the flywheel speed
   * @param speed taken in units of RPM
   */
  public void setFlywheelSpeed(double rpm) {
    leftFlywheelMotor.set(ControlMode.Velocity, rpmToTalonFXUnits(rpm));
  }

  /**
   * NOT YET IMPLEMENTED
   * @return the speed of the motor in units of RPM
   */
  public double getFlywheelSpeed() {
    return talonFXUnitsToRpm(leftFlywheelMotor.getClosedLoopTarget());
  }

  /**
   * Set the accelerator wheel speed
   * @param rpm taken in units of RPM
   */
  public void setAcceleratorRPM(double rpm) {
    acceleratorWheelMotor.set(ControlMode.Velocity, rpmToTalonFXUnits(rpm));

  }

  /**
   * NOT YET IMPLEMENTED
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
    // This method will be called once per scheduler run
    
  }

  private double rpmToTalonFXUnits(double rpm) {
    return rpm * 2048 / 600;
  }

  private double talonFXUnitsToRpm(double talonFXUnit) {
    return (talonFXUnit / 2048) * 10 * 60;
  }

  private void reinitTalonFx(TalonFX talonFX) {
    talonFX.configFactoryDefault();
    talonFX.configNeutralDeadband(0.001);
    talonFX.setNeutralMode(NeutralMode.Coast);
    talonFX.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 45, 1));
    talonFX.configSelectedFeedbackSensor(
      TalonFXFeedbackDevice.IntegratedSensor,
      Constants.Shooter.kPIDLoopIdx, 
      Constants.Shooter.kTimeoutMs
    );
    talonFX.configNominalOutputReverse(0, Constants.Shooter.kTimeoutMs);
    talonFX.configPeakOutputForward(1, Constants.Shooter.kTimeoutMs);
    talonFX.configPeakOutputReverse(-1, Constants.Shooter.kTimeoutMs);
  }

  private void setTalonFXPidGains(TalonFX talonFX) {
    talonFX.config_kF(Constants.Shooter.kPIDLoopIdx, Constants.Shooter.FLYWHEEL_KF, Constants.Shooter.kTimeoutMs);
    talonFX.config_kP(Constants.Shooter.kPIDLoopIdx, Constants.Shooter.FLYWHEEL_KP, Constants.Shooter.kTimeoutMs);
    talonFX.config_kI(Constants.Shooter.kPIDLoopIdx, Constants.Shooter.FLYWHEEL_KI, Constants.Shooter.kTimeoutMs);
    talonFX.config_kD(Constants.Shooter.kPIDLoopIdx, Constants.Shooter.FLYWHEEL_KD, Constants.Shooter.kTimeoutMs);
  }
}
