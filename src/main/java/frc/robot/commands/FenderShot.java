// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;

public class FenderShot extends CommandBase {
  public final Indexer mIndexer;
  public final Hood mHood;
  public final Flywheel mFlywheel;
  /** Creates a new FenderShot. */
  public FenderShot(Indexer indexer, Hood hood, Flywheel flywheel) {
    // Use addRequirements() here to declare subsystem dependencies.
    mIndexer = indexer;
    mFlywheel = flywheel;
    mHood = hood;
    addRequirements(indexer, hood, flywheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // set hood position
    // position is between 0 and 1
    mHood.SetHoodPosition(0.1);
    // get flywheel up to speed
    mFlywheel.SetSpeed(Constants.FlywheelSpeed);
    // get controller value
    Debouncer mDebouncer = new Debouncer(0.1, DebounceType.kBoth);

    // calculates if the button is true or not
    if (mDebouncer.calculate(RobotContainer.driverJoyStick.getXButtonPressed())) {
      // put indexer subsystem interface
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
