// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveByDistance extends PIDCommand {
  /** Creates a new DriveByDistance. */
  DriveTrain driveTrain;
  public DriveByDistance(double targetDistance, DriveTrain mDriveTrain) {  
    super(
      // The controller that the command will use
      new PIDController(0.4, 0, 0),
      // This should return the measurement
      mDriveTrain::getAverageEncoderDistance,
      // This should return the setpoint (can also be a constant)
      targetDistance + mDriveTrain.getAverageEncoderDistance(),
      // This uses the output
      output -> mDriveTrain.arcadeDrive(-output + Math.copySign(getFeedforward(), -output), 0),
      // Requirements
      mDriveTrain
    );
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(0.1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }

  private static double getFeedforward() {
    double goalVoltage = 2.5; 
    double percentOutput = goalVoltage / RobotController.getBatteryVoltage();
    return percentOutput;
  }
}
