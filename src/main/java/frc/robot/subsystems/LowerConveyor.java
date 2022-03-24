// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.lowerCon.*;

public class LowerConveyor extends SubsystemBase {
  /** Creates a new LowerConveyer. */
  CANSparkMax lowConMotor;
  TalonFX pooperMotor;
  DigitalInput lowConBreakBeam;
  DigitalInput pooperBreakBeam;

  public LowerConveyor() {
    lowConMotor = new CANSparkMax(LOWER_CON_MOTOR_CAN_ID, MotorType.kBrushless);
    lowConMotor.setSmartCurrentLimit(20);
    lowConMotor.enableVoltageCompensation(12.0);
    pooperMotor = new TalonFX(POOPER_MOTOR_CAN_ID);
    reinitTalonFx(pooperMotor);    
    pooperMotor.setInverted(true);
    lowConBreakBeam = new DigitalInput(LOWER_CON_BREAK_BEAM_DIO_PORT);
    pooperBreakBeam = new DigitalInput(POOPER_BREAK_BEAM_DIO_PORT);
  }

  /**
   * 1.0 is intaking, -1.0 is pooping
   * 
   * @param speed
   */
  public void setPooperPercentOutput(double speed) {
    pooperMotor.set(ControlMode.PercentOutput ,speed);
  }

  /**
   * 1.0 is intaking, -1.0 is reversing
   * 
   * @param speed
   */
  public void setLowerConveyorPercentOutput(double speed) {
    lowConMotor.set(speed);
  }

  /**
   * Broken will return true
   * 
   * @return
   */
  public boolean getLowConBreakBeam() {
    return !lowConBreakBeam.get();
  }

  /**
   * Broken will return true
   * 
   * @return
   */
  public boolean getPooperBreakBeam() {
    return !pooperBreakBeam.get();
  }

  private void reinitTalonFx(TalonFX talonFX) {
    talonFX.configFactoryDefault();
    talonFX.configNeutralDeadband(0.001);
    talonFX.setNeutralMode(NeutralMode.Coast);
    talonFX.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 
    POOPER_CURRENT_LIMIT, POOPER_CURRENT_LIMIT + 5, 1));
    talonFX.configNominalOutputReverse(0, 30);
    talonFX.configPeakOutputForward(1, 30);
    talonFX.configPeakOutputReverse(-1, 30);
  }

}
