package frc.robot.commands;

import java.util.Collections;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.FieldConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.RobotContainer.AutoPosition;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.LowerConveyor;
import frc.robot.subsystems.UpperConveyor;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.DriveTrain;
import frc.robot.util.GeomUtil;

public class PoopTwoAuto extends SequentialCommandGroup{
    private static final Pose2d ourCargoAimed = calcAimedPose(FieldConstants.cargoB.transformBy(GeomUtil.transformFromTranslation(-0.5, 0.0)));

    private static final Pose2d firstCargoTurnPosition = new Pose2d(
        ourCargoAimed.getTranslation(),
        Rotation2d.fromDegrees(-120.0));
    private static final Pose2d firstCargoPosition =
        FieldConstants.cargoC.transformBy(
            new Transform2d(new Translation2d(), Rotation2d.fromDegrees(90.0)));
    private static final Pose2d secondCargoTurnPosition =
        FieldConstants.cargoC.transformBy(
            new Transform2d(new Translation2d(), Rotation2d.fromDegrees(-90.0)));
    private static final Pose2d secondCargoPosition =
        FieldConstants.cargoA.transformBy(
            new Transform2d(new Translation2d(), Rotation2d.fromDegrees(-90.0)));
    private static final Pose2d shootPosition =
        FieldConstants.referenceA.transformBy(new Transform2d(
            new Translation2d(2.0, 1.0), Rotation2d.fromDegrees(-120.0)));

    public PoopTwoAuto(
        DriveTrain driveTrain, 
        Intake intake, 
        LowerConveyor lowerConveyor, 
        UpperConveyor upperConveyor, 
        Shooter shooter, 
        Vision vision,
        ColorSensor colorSensor) {
        
        Pose2d start = AutoPosition.TARMAC_A.getPose();
        InstantCommand resetOdometryCommand = new InstantCommand(() -> driveTrain.resetOdometry(start));
        LoggingRamseteCommand driveToOurBall = new LoggingRamseteCommand(driveTrain, start, Collections.emptyList(), ourCargoAimed, false);

        // Turn away from wall
        TurnByAngle turnTowardsFirstBall = new TurnByAngle(90, driveTrain);
        LoggingRamseteCommand driveToFirstBall = new LoggingRamseteCommand(driveTrain, firstCargoTurnPosition, Collections.emptyList(), firstCargoPosition, false);
        TurnByAngle turnTowardsSecondball = new TurnByAngle(180, driveTrain);
        LoggingRamseteCommand driveToSecondnBall = new LoggingRamseteCommand(driveTrain, secondCargoTurnPosition, Collections.emptyList(), secondCargoPosition, false);
        LoggingRamseteCommand driveToHanger = new LoggingRamseteCommand(driveTrain, secondCargoPosition, Collections.emptyList(), shootPosition, true);

        addCommands(
            resetOdometryCommand,
                // Drive to first ball and shoot preload/first ball. Spin up shooter on the way
            deadline(driveToOurBall,
                new IntakeOut(intake), 
                new IntakeRun(intake),
                new Index(lowerConveyor, upperConveyor, colorSensor),
                new SetFlywheelToLimelightShot(shooter, vision)),
            deadline(new WaitCommand(0.25).andThen(new ShootBall(upperConveyor, lowerConveyor, colorSensor).withTimeout(1)),
                new SetFlywheelToLimelightShot(shooter, vision),
                new DriveWithLimelight(driveTrain, vision)),
            turnTowardsFirstBall.withTimeout(1),
            deadline(driveToFirstBall,  
                    new IntakeRun(intake),
                    new Index(lowerConveyor, upperConveyor, colorSensor, () -> false, () -> true)),
            // turnTowardsSecondball.withTimeout(1),
            deadline(driveToSecondnBall,  
                    new IntakeRun(intake),
                    new Index(lowerConveyor, upperConveyor, colorSensor, () -> false, () -> true)),
            driveToHanger, 
            new IntakeIn(intake),
            new Purge(intake, lowerConveyor, upperConveyor, shooter).withTimeout(3)
        );
    }

    public static Pose2d calcAimedPose(Pose2d pose) {
        Translation2d vehicleToCenter =
            FieldConstants.hubCenter.minus(pose.getTranslation());
        Rotation2d targetRotation =
            new Rotation2d(vehicleToCenter.getX(), vehicleToCenter.getY());
        targetRotation = targetRotation.plus(Rotation2d.fromDegrees(180));
        return new Pose2d(pose.getTranslation(), targetRotation);
    }
}
