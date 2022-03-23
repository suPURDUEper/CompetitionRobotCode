package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Climber;

public class MoveToNextBar extends SequentialCommandGroup {
  public MoveToNextBar(Climber climber, BooleanSupplier continueConfirmation) {
    addCommands(
      parallel(new ClimberUp(climber), new InstantCommand(climber::climberTilt)),
      new InstantCommand(climber::climberStraight),
      parallel(new WaitCommand(.5), new WaitUntilCommand(continueConfirmation)),
      new ClimberDown(climber)
    );
  }
}
