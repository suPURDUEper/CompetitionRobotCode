// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LowerConveyor extends SubsystemBase {
  /** Creates a new LowerConveyer. */
  CANSparkMax lowConMotor;
  TalonSRX pooperMotor;
  DigitalInput lowConBreakBeam;
  DigitalInput pooperBreakBeam;
  /** Color Sensor and I2C setup */
  private final I2C.Port i2cPort = I2C.Port.kMXP;
  private final ColorSensorV3 colorSensor;
  private final ColorMatch mColorMatcher;
  /**
   * Create the colors to store in the colormatcher to compare the ball color
   * against
   */
  private final Color kBlueTarget = new Color(0.143, 0.427, 0.429);
  private final Color kGreenTarget = new Color(0.197, 0.561, 0.240);
  private final Color kRedTarget = new Color(0.561, 0.232, 0.114);
  private final Color kYellowTarget = new Color(0.361, 0.524, 0.113);
  /** Make A Detected Color Variable which is reset every Period */
  private Color detectedColor;
  private final NetworkTableEntry mDetectedColor;

  public LowerConveyor() {
    // Color Sensor and Macther
    colorSensor = new ColorSensorV3(i2cPort);
    mColorMatcher = new ColorMatch();
    // These are the defaul RGB values given for the color red from rev robotics
    // example
    // Init detected color
    detectedColor = colorSensor.getColor();
    mColorMatcher.addColorMatch(kRedTarget);
    mColorMatcher.addColorMatch(kBlueTarget);
    mColorMatcher.addColorMatch(kGreenTarget);
    mColorMatcher.addColorMatch(kYellowTarget);
    lowConMotor = new CANSparkMax(Constants.lowerCon.LowConMotor, MotorType.kBrushless);
    lowConMotor.enableVoltageCompensation(12.0);
    pooperMotor = new TalonSRX(Constants.lowerCon.PooperMotor);

    lowConBreakBeam = new DigitalInput(Constants.lowerCon.LowConBreakBeam);
    pooperBreakBeam = new DigitalInput(Constants.lowerCon.PooperBreakBeam);
    mDetectedColor = Shuffleboard.getTab("Limelight Aim").add("Ball Color", "No Ball").getEntry();
  }

  public boolean HasTeamBall() {
    ColorMatchResult match = mColorMatcher.matchColor(detectedColor);
    // if the value is no where close to the desired
    // then null will be returned
    if (match.color != null) {
      if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
        if (match.color == kRedTarget) {
          return true;
        }
      } else if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
        if (match.color == kBlueTarget) {
          return true;
        }
      }
    }
    return false;
  }

  public boolean ColorSensorHasTarget() {
    if (mColorMatcher.matchColor(detectedColor) != null) {
      return true;
    }
    return false;
  }

  /**
   * 1.0 is intaking, -1.0 is pooping
   * 
   * @param speed
   */
  public void setPooperPercentOutput(double speed) {
    pooperMotor.set(ControlMode.PercentOutput, speed);
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

  /**
   * Check if the blue ball is in the intake
   * 
   * @return boolean
   */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    detectedColor = colorSensor.getColor();
    if (ColorSensorHasTarget()) {
      if (HasTeamBall()) {
        mDetectedColor.forceSetString("Team Ball");
      } else if (!HasTeamBall()) {
        mDetectedColor.forceSetString("Enemy Ball");
      }
    } else {
      mDetectedColor.forceSetString("No Ball");
    }
  }
}
