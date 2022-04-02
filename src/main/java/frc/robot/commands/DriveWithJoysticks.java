// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

public class DriveWithJoysticks extends CommandBase {
  private final DriveTrain driveTrain;

  /** Creates a new DriveWithJoysticks. */
  public DriveWithJoysticks(DriveTrain dt) {
    driveTrain = dt;
    addRequirements(driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = Constants.DriveTrain.BoostInactive;
    if (RobotContainer.driverJoyStick.getLeftBumper())
      speed = Constants.DriveTrain.BoostActive;
      // drive math currently not working
    // driveTrain.driveWithJoysticks(
    //     DriveController.getThrottleMap(RobotContainer.driverJoyStick.getLeftY(), speed),
    //     DriveController.getTurnMap(RobotContainer.driverJoyStick.getRightX(), speed));
    double throttleInput, turnInput;
    if (Robot.isReal()) {
      throttleInput = RobotContainer.driverJoyStick.getLeftY();
      turnInput = RobotContainer.driverJoyStick.getRightX();
    } else {
      // WASD for driving robot in simulator
      turnInput = -1 * RobotContainer.driverJoyStick.getRawAxis(0);
      throttleInput = -1 * RobotContainer.driverJoyStick.getRawAxis(1);
    }
    driveTrain.driveWithJoysticks(speed * squareJoystick(-throttleInput), speed * squareJoystick(turnInput));
  }

  private double squareJoystick(double original) {
    return Math.copySign(Math.pow(original, 2), original);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
