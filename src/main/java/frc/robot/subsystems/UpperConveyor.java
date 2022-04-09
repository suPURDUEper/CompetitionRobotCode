// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory

package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class UpperConveyor extends SubsystemBase {
  CANSparkMax ConveyorMotor;
  DigitalInput UpperConveyorSensor;
  DigitalInput MidConveyorSensor;

  /** Creates a new UpperConveyer. */
  public UpperConveyor() {
    ConveyorMotor = new CANSparkMax(Constants.UpperCon.UpperConMotor, MotorType.kBrushless);
    ConveyorMotor.setInverted(true);
    ConveyorMotor.setSmartCurrentLimit(20);
    ConveyorMotor.setIdleMode(IdleMode.kBrake);
    ConveyorMotor.enableVoltageCompensation(12.0);
    ConveyorMotor.burnFlash();
    UpperConveyorSensor = new DigitalInput(Constants.UpperCon.UpperConBreakBeam);
    MidConveyorSensor = new DigitalInput(Constants.UpperCon.MidConBreakBeam);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public void setPercentOutput(double speed) {
    ConveyorMotor.set(speed);
  }

  public boolean hasTopBall() {
    return !UpperConveyorSensor.get();
  }
  public boolean hasTwoBalls() {
    return !MidConveyorSensor.get() && hasTopBall();
  }
}
