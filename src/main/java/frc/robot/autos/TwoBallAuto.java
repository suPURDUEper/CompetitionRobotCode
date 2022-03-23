package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveByDistance;
import frc.robot.commands.Index;
import frc.robot.commands.IntakeIn;
import frc.robot.commands.IntakeOut;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LowerConveyor;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.UpperConveyor;
import frc.robot.subsystems.Vision;

public class TwoBallAuto extends SequentialCommandGroup{
  public TwoBallAuto(DriveTrain driveTrain, Intake intake, LowerConveyor lowerCon, UpperConveyor upperCon, Shooter shooter, Vision vision) {
    addCommands(
      new IntakeOut(intake),
      new WaitCommand(0.1),
      parallel(
        new Index(lowerCon, upperCon), 
        new DriveByDistance(1, driveTrain)),
      new IntakeIn(intake),
      new SpinUpAndFireTwoBalls(driveTrain, lowerCon, upperCon, shooter, vision),
      new DriveByDistance(0.3, driveTrain)
    );
  }
}
