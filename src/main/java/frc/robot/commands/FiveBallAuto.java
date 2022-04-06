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
import frc.robot.RobotContainer.AutoPosition;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LowerConveyor;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.UpperConveyor;
import frc.robot.subsystems.Vision;
import frc.robot.util.GeomUtil;

public class FiveBallAuto extends SequentialCommandGroup {

    public FiveBallAuto(
        DriveTrain driveTrain, 
        Intake intake, 
        LowerConveyor lowerConveyor, 
        UpperConveyor upperConveyor, 
        Shooter shooter, 
        Vision vision,
        ColorSensor colorSensor) {

            // Drive from start to first ball
            Pose2d start = AutoPosition.TARMAC_D.getPose();
            Pose2d firstBallAimed = calcAimedPose(FieldConstants.cargoE.transformBy(GeomUtil.transformFromTranslation(-0.5, 0.0)));
            LoggingRamseteCommand driveToFirstBall = new LoggingRamseteCommand(driveTrain, start, Collections.emptyList(), firstBallAimed);

            // Turn away from wall
            TurnByAngle turnToBall2 = new TurnByAngle(-120, driveTrain);

            // Drive to second ball
            Pose2d startToDriveTo2ndBall = firstBallAimed.transformBy(new Transform2d(new Translation2d(), new Rotation2d(-120)));
            Pose2d secondBallAimed = calcAimedPose(FieldConstants.cargoD.transformBy(new Transform2d(new Translation2d(-0.2, 0.2), new Rotation2d())));
            LoggingRamseteCommand driveToSecondBall = new LoggingRamseteCommand(driveTrain, startToDriveTo2ndBall, Collections.emptyList(), secondBallAimed);

            // Drive to terminal
            Pose2d terminalCargoPosition = FieldConstants.cargoG.transformBy(new Transform2d(new Translation2d(0.5, 0.0), Rotation2d.fromDegrees(180.0)));
            Translation2d terminalCargoApproachPosition = terminalCargoPosition.transformBy(GeomUtil.transformFromTranslation(-0.8, 0.0)).getTranslation();
            LoggingRamseteCommand driveToTerminalBall = new LoggingRamseteCommand(driveTrain, secondBallAimed, Collections.emptyList(), terminalCargoPosition);

            // Drive to final shot location
            Pose2d finalShotLocation = new Pose2d(); // todo
            LoggingRamseteCommand driveToFinalShot = new LoggingRamseteCommand(driveTrain, terminalCargoPosition, Collections.emptyList(), finalShotLocation);

            driveTrain.addTrajectoryToDashboard(driveToFirstBall.getTrajectory());
            driveTrain.addTrajectoryToDashboard(driveToSecondBall.getTrajectory());
            InstantCommand resetOdometryCommand = new InstantCommand(() -> driveTrain.resetOdometry(start));

            addCommands(
                resetOdometryCommand,
                new IntakeOut(intake),

                // Drive to first ball
                deadline(driveToFirstBall, 
                    new IntakeRun(intake),
                    new Index(lowerConveyor, upperConveyor, colorSensor)),

                // Shoot preload and first ball
                deadline(new WaitCommand(0.25).andThen(new ShootBall(upperConveyor, lowerConveyor).withTimeout(1.5)),
                    new SetFlywheelToLimelightShot(shooter, vision)),

                // Drive to second ball
                turnToBall2.withTimeout(0.5), 
                deadline(driveToSecondBall, 
                    new IntakeRun(intake),
                    new Index(lowerConveyor, upperConveyor, colorSensor)),

                // Shoot second ball
                deadline(new WaitCommand(0.25).andThen(new ShootBall(upperConveyor, lowerConveyor).withTimeout(.75)),
                    new SetFlywheelToLimelightShot(shooter, vision),
                    new DriveWithLimelight(driveTrain, vision)),

                // Drive to terminal with slight pause
                deadline(driveToTerminalBall.andThen(new WaitCommand(0.5)), 
                    new IntakeRun(intake), 
                    new Index(lowerConveyor, upperConveyor, colorSensor)),

                // Drive to final shot location
                parallel(
                    new SetFlywheelToLimelightShot(shooter, vision),
                    deadline(new DriveWithLimelight(driveTrain, vision, () -> 0.8).withTimeout(1),
                        new Index(lowerConveyor, upperConveyor, colorSensor))
                    .andThen(new ShootBall(upperConveyor, lowerConveyor))
                )


            );
            addRequirements(driveTrain, intake, upperConveyor, lowerConveyor, shooter);
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
