// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hood extends SubsystemBase {
  private final double max = 1.75;
  private final double  deadbandMax = 1.7;
  private final double center = 1.5;
  private final double deadbandMin = 1.2;
  private final double min = 1.1;
  Servo HoodServo;

  /** Creates a new Hood. */
  public Hood() {
    HoodServo = new Servo(Constants.HOOD_SERVO_CHANNEL);
    HoodServo.setBounds(max, deadbandMax, center, deadbandMin, min);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void SetHoodPosition(double position) {
    // 0 - 1. 0 fully extended and 1 is fully retracted
    HoodServo.setSpeed(position);
  }
}
