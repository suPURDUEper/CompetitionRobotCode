// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.LowerConveyor;
import frc.robot.subsystems.UpperConveyor;

public class Index extends CommandBase {
  /** Creates a new IntakeRun. */
  private final LowerConveyor lowCon;
  private final UpperConveyor upperCon;
  private final ColorSensor colorSensor;
  private final BooleanSupplier reversePooper;

  private long pooperStartTimeUs = 0;
  private static final int POOPER_REVERSE_TIME_MS = 1000;

  public Index(LowerConveyor mLowerConveyor, UpperConveyor mUpperConveyor, ColorSensor colorSensor) {
    this(mLowerConveyor, mUpperConveyor, colorSensor, () -> false);
  }

  public Index(LowerConveyor mLowerConveyor, UpperConveyor mUpperConveyor, ColorSensor colorSensor, BooleanSupplier reversePooper) {
    // Use addRequirements() here to declare subsystem dependencies.
    lowCon = mLowerConveyor;
    addRequirements(mLowerConveyor);
    upperCon = mUpperConveyor;
    addRequirements(mUpperConveyor);
    this.colorSensor = colorSensor;
    this.reversePooper = reversePooper;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Run upper conveyor if we have no balls, stop otherwise
    upperCon.setPercentOutput(upperCon.hasTopBall() ? 0 : 1);

    // Run lower conveyor always, command will end when we have two balls
    lowCon.setLowerConveyorPercentOutput(1);

    // // Run pooper as long as we don't have two balls.
    // // Switch direction momentarily for wrong color ball
    // if (colorSensor.HasWrongBall()) {
    //   // Switch the direction of the pooper for a second
    //   pooperStartTimeUs = RobotController.getFPGATime();
    // }
    // long pooperEndReverseTime = pooperStartTimeUs + (1000 * POOPER_REVERSE_TIME_MS);
    // int direction = (RobotController.getFPGATime() < pooperEndReverseTime) ? -1 : 1;
    int direction = 1;
    if (reversePooper != null && reversePooper.getAsBoolean()) {
      direction = -1;
    }
    if (colorSensor.HasWrongBall()) {
      direction = -1;
    }
    lowCon.setPooperPercentOutput(direction * 0.8 * 0.7);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    lowCon.setPooperPercentOutput(0);
    lowCon.setLowerConveyorPercentOutput(0);
    upperCon.setPercentOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return upperCon.hasTwoBalls();
  }
}
