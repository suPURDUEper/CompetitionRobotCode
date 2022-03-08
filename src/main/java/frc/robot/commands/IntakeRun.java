// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LowerConveyor;
import frc.robot.subsystems.UpperConveyor;

public class IntakeRun extends CommandBase {
  /** Creates a new IntakeRun. */
  private final Intake intake;
  private final LowerConveyor lowCon;
  private final UpperConveyor upperCon;

  public IntakeRun(Intake mIntake, LowerConveyor mLowerConveyor, UpperConveyor mUpperConveyor) {
    // Use addRequirements() here to declare subsystem dependencies.
    intake = mIntake;
    addRequirements(mIntake);
    lowCon = mLowerConveyor;
    addRequirements(mLowerConveyor);
    upperCon = mUpperConveyor;
    addRequirements(mUpperConveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    System.out.println(upperCon.getUpperConBreakBeam());
    if (upperCon.getUpperConBreakBeam()) {
      upperCon.ConveyorMotorSet(0);
      if (upperCon.HasTwoBalls()) {
        lowCon.LowConMotorSet(0);
        intake.IntakeMotorSet(0);
        intake.IndexerMotorSet(0);
      }
    } else {
      upperCon.ConveyorMotorSet(1);
      intake.IndexerMotorSet(1);
      intake.IntakeMotorSet(1);
      // figure out which direction the motors need to run
      // getting the break beam, and color sensor values
      // getLowConBreakBeam(), HasTeamBall()
      lowCon.PooperMotorSet(.8);
      lowCon.LowConMotorSet(1);
      if (lowCon.ColorSensorHasTarget()) {
        if (lowCon.HasTeamBall()) {
          lowCon.PooperMotorSet(0.8);
          // System.out.print("good ball");
        } else {
          lowCon.PooperMotorSet(-1.0);
          // System.out.print("bad ball");
        }

      } else {
        lowCon.PooperMotorSet(.8);
        // System.out.print("no ball");
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.IndexerMotorSet(0);
    intake.IntakeMotorSet(0);
    lowCon.PooperMotorSet(0);
    lowCon.LowConMotorSet(0);
    upperCon.ConveyorMotorSet(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
