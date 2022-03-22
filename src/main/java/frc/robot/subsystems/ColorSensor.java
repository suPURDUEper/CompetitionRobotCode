// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PicoColorSensor;
import frc.robot.PicoColorSensor.RawColor;

public class ColorSensor extends SubsystemBase {
  PicoColorSensor colorSensor;
  RawColor rawColor;
  Color color;
  double ir;

  /** Creates a new ColorSensor. */
  public ColorSensor() {
    colorSensor = new PicoColorSensor();
    rawColor = new RawColor();
    color = new Color(0, 0, 0);
    ir = rawColor.ir;

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Color getColor() {
    rawColor = colorSensor.getRawColor0();
    double colorsum = rawColor.red + rawColor.green + rawColor.blue;
    final Color color = new Color(rawColor.red/colorsum, rawColor.green/colorsum, rawColor.blue/colorsum);
    System.out.println("RED: " + color.red + " GREEN: " + color.green + " BLUE: " + color.blue + " IR: " + ir);
    return color;
  }
  public boolean HasWrongBall() {
    
    // if the value is no where close to the desired
    // then null will be returned
    if (ir <= 5) {
      if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {

      }
    }
    return false;
  } 
}
