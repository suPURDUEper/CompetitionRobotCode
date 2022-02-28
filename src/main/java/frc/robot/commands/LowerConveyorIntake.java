// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LowerConveyor;

public class LowerConveyorIntake extends CommandBase {
  private final LowerConveyor lowCon;

  /** Creates a new LowerConveyorIntake. */
  public LowerConveyorIntake(LowerConveyor mLowCon) {
    // Use addRequirements() here to declare subsystem dependencies.
    lowCon = mLowCon;
    addRequirements(mLowCon);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // figure out which direction the motors need to run
    // getting the break beam, and color sensor values
    // getLowConBreakBeam(), HasTeamBall()
    lowCon.PooperMotorSet(.8);
    lowCon.LowConMotorSet(1);
   /* if (lowCon.ColorSensorHasTarget()) {
      if (lowCon.HasTeamBall()) {
        lowCon.PooperMotorSet(1.0);
      } else {
        lowCon.PooperMotorSet(-1.0);
      }
      
    } else {
      lowCon.PooperMotorSet(1.0);
    }*/


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    lowCon.PooperMotorSet(0);
    lowCon.LowConMotorSet(0);
    //lowCon.PooperMotorSet(0);
    //lowCon.LowConMotorSet(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
