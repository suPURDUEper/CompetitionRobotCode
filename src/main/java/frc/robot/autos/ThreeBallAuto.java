package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveByDistance;
import frc.robot.commands.Index;
import frc.robot.commands.IntakeIn;
import frc.robot.commands.IntakeOut;
import frc.robot.commands.IntakeRun;
import frc.robot.commands.TurnByAngle;
import frc.robot.subsystems.*;

public class ThreeBallAuto extends SequentialCommandGroup {

  public ThreeBallAuto(DriveTrain driveTrain, Intake intake, LowerConveyor lowerCon, UpperConveyor upperCon, Shooter shooter, Vision vision) {
    addCommands(
      new IntakeOut(intake),
      new WaitCommand(0.1),
      race(
        new Index(lowerCon, upperCon), 
        new IntakeRun(intake),
        new DriveByDistance(1, driveTrain)),
      new SpinUpAndFireTwoBalls(driveTrain, lowerCon, upperCon, shooter, vision),
      new DriveByDistance(-0.3, driveTrain),
      new WaitCommand(.1),
      new TurnByAngle(-108.5, driveTrain),
      new WaitCommand(0.1),
      new IntakeOut(intake),
      race(
        new Index(lowerCon, upperCon), 
        new IntakeRun(intake),
        new DriveByDistance(3.0, driveTrain)),
      new WaitCommand(0.1),
      new TurnByAngle(-58, driveTrain),
      new IntakeIn(intake),
      new SpinUpAndFireTwoBalls(driveTrain, lowerCon, upperCon, shooter, vision)
    );
  }
}
