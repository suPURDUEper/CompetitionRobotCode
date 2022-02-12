// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

public class DriveWithBoost extends CommandBase {
  private final DriveTrain driveTrain;
  /** Creates a new DriveWithBoost. */
  public DriveWithBoost(DriveTrain dt) {
    driveTrain = dt;
    addRequirements(driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    DriveTrain.boost = Constants.DriveTrain.BoostActive;
    driveTrain.driveWithJoysticks(RobotContainer.driverJoyStick, Constants.DriveTrain.DriveTrainSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DriveTrain.boost = Constants.DriveTrain.BoostInactive;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
