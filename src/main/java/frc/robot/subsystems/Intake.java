// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * The intake class envelops the 4 motors which run the intake as well as the color sensor to sense the color of ball.
 * We also use break beams to determine whether or not the clip is fully loaded. In 2022's game a fully loaded clip is 2 balls.
 * 
 * If a red ball comes into the intake, we sense the color and if that is red then we reverse the indexer motor and send it flying
 * out the 'pooper'. Blue balls get to pass freely through.
 * 
 * If a blue ball comes before a red ball, then it get stored up in the top most part of the intake/conveyor. This allows red balls
 * to still be pooped out.
 * 
 * Once two blue balls fill the intake, we shut down the ability to use the intake except for in the case of shooting.
 */
public class Intake extends SubsystemBase {
  /** Pneumatics */
  private final DoubleSolenoid leftIntakeSolenoid;
  private final DoubleSolenoid rightIntakeSolenoid;
  /** Indexer motors */
  private final CANSparkMax indexerMotor;
  /** Intake Motor */
  private final WPI_TalonFX intakeMotor;

  public Intake() {
    // indexer motors
    indexerMotor = new CANSparkMax(Constants.Intake.INDEXER_MOTOR_ID, MotorType.kBrushless);
    // index motors
    intakeMotor = new WPI_TalonFX(Constants.Intake.INTAKE_MOTOR_TALON_ID);
    intakeMotor.configFactoryDefault();
    // Pneumatics
    leftIntakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
    rightIntakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4, 5);
    leftIntakeSolenoid.set(DoubleSolenoid.Value.kForward);
    rightIntakeSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  /**
   * Expects kFoward, kReverse or False from DoubleSolenoid.Value Enum
   * @param value DoubleSolenoid.Value type
   */
  public void IntakeSet(DoubleSolenoid.Value value) {
    leftIntakeSolenoid.set(value);
    rightIntakeSolenoid.set(value);
  }

  /**
   * 1.0 is full forward, -1.0 is full backward
   * @param speed -1.0 to 1.0
   */
  public void IndexerMotorSet(double speed) {
    /// no need to set left indexer
    /// left indexer follows right indexer
    indexerMotor.set(speed);
  }

  /**
   * 1.0 if full forward, -1.0 is full backward
   * @param speed -1.0 to 1.0
   */
  public void IntakeMotorSet(double speed) {
    intakeMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // update the detected color every period
    // detectedColor = colorSensor.getColor();
  }
}
