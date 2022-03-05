// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.UpperConveyor;

public class UpperConveyorIntake extends CommandBase {
  /** Creates a new UpperConveyorIntake. */
  private final UpperConveyor upperCon;
  public UpperConveyorIntake(UpperConveyor mUpperCon) {
    upperCon = mUpperCon;
    addRequirements(mUpperCon);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(upperCon.getUpperConBreakBeam());
    if(upperCon.getUpperConBreakBeam()){
    upperCon.ConveyorMotorSet(0);
    } else {
      upperCon.ConveyorMotorSet(1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    upperCon.ConveyorMotorSet(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
