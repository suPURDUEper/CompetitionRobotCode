// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.util.Color;
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
  private final CANSparkMax indexerMoter1;
  private final CANSparkMax indexerMotor2;
  private final CANSparkMax indexerMotor3;
  /** Intake Motor */
  private final CANSparkMax intakeMotor;
  /** Color Sensor and I2C setup */
  // private final I2C.Port i2cPort = I2C.Port.kOnboard;
  // private final ColorSensorV3 colorSensor;
  private final ColorMatch mColorMatcher;
  /** Create the colors to store in the colormatcher to compare the ball color against */
  private final Color kBlueTarget = new Color(0.143, 0.427, 0.429);
  private final Color kGreenTarget = new Color(0.197, 0.561, 0.240);
  private final Color kRedTarget = new Color(0.561, 0.232, 0.114);
  private final Color kYellowTarget = new Color(0.361, 0.524, 0.113);
  /** Make A Detected Color Variable which is reset every Period */
  private Color detectedColor;

  public Intake() {
    // Color Sensor and Macther
    // colorSensor = new ColorSensorV3(i2cPort);
    mColorMatcher = new ColorMatch();
    // These are the defaul RGB values given for the color red from rev robotics
    // example
    mColorMatcher.addColorMatch(kRedTarget);
    // Init detected color
    // detectedColor = colorSensor.getColor();
    // intake motor
    intakeMotor = new CANSparkMax(Constants.IntakeMotor, MotorType.kBrushless);
    // index motors
    indexerMoter1 = new CANSparkMax(Constants.IndexerMotor1, MotorType.kBrushless);
    indexerMoter1.setInverted(false);
    indexerMotor2 = new CANSparkMax(Constants.IndexerMotor2, MotorType.kBrushless);
    indexerMotor2.setInverted(false);
    indexerMotor3 = new CANSparkMax(Constants.IndexerMotor3, MotorType.kBrushless);
    indexerMotor3.setInverted(false);
    // Pneumatics
    leftIntakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
    rightIntakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4, 5);
    leftIntakeSolenoid.set(DoubleSolenoid.Value.kForward);
    rightIntakeSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  /**
   * Check if a red ball is in the intake.
   * @return boolean 
   */
  public boolean HasRedBall() {
    ColorMatchResult match = mColorMatcher.matchColor(detectedColor);
    // if the value is no where close to the desired
    // then null will be returned
    if (match.color != null) {
      if (match.color == kRedTarget) {
        return true;
      }
    }
    return false;
  }

  /**
   * Check if the blue ball is in the intake
   * @return boolean
   */
  public boolean HasBlueBall() {
    ColorMatchResult match = mColorMatcher.matchColor(detectedColor);
    // if the value is no where close to the desired
    // then null will be returned
    if (match.color != null) {
      if (match.color == kBlueTarget) {
        return true;
      }
    }
    return false;
  }

  /**
   * Toggles the intake between either in -> out or out -> in
   * @param controller the operator controller
   */
  public void ToggleIntake(XboxController controller) {
    if (controller.getBButton()) {
      leftIntakeSolenoid.toggle();
      rightIntakeSolenoid.toggle();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // update the detected color every period
    // detectedColor = colorSensor.getColor();
  }
}
