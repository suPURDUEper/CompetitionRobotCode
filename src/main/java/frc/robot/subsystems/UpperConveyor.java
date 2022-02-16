// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory

package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class UpperConveyor extends SubsystemBase {
  CANSparkMax ConveyorMotor;
  DigitalInput UpperConveyorSensor;
  public static final int mUpperConveyorSensor = 0;

  /** Creates a new UpperConveyer. */
  public UpperConveyor() {
    ConveyorMotor = new CANSparkMax(Constants.UpperCon.UpperConMotor, MotorType.kBrushless);
    UpperConveyorSensor = new DigitalInput(mUpperConveyorSensor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public boolean getUpperConBreakBeam() {
    return !UpperConveyorSensor.get();
  }

  public void ConveyorMotorSet(double speed) {

    ConveyorMotor.set(speed);
  }

  public boolean IsUpperConveyorOpen() {
    return !UpperConveyorSensor.get();
  }
}
