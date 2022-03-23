package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveWithLimelight;
import frc.robot.commands.SetFlywheelToLimelightShot;
import frc.robot.commands.ShootBall;
import frc.robot.subsystems.*;

public class SpinUpAndFireTwoBalls extends ParallelRaceGroup {
  public SpinUpAndFireTwoBalls(DriveTrain driveTrain, LowerConveyor lowerCon, UpperConveyor upperCon, Shooter shooter, Vision vision) {
    addCommands(
      new DriveWithLimelight(driveTrain, vision),
      new SetFlywheelToLimelightShot(shooter, vision),
      sequence(
        new WaitCommand(0.5),
        new ShootBall(upperCon, lowerCon, shooter::isShooterAtSpeed).withTimeout(2)));
    }
}
