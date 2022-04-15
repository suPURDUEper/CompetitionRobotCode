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
            LoggingRamseteCommand driveToFirstBall = new LoggingRamseteCommand(driveTrain, start, Collections.emptyList(), firstBallAimed, false);

            // Turn away from wall
            TurnByAngle turnTowardsSecondBall = new TurnByAngle(-120, driveTrain);

            // Drive to second ball
            Pose2d secondBallPathStart = firstBallAimed.transformBy(new Transform2d(new Translation2d(), new Rotation2d(-120)));
            Pose2d secondBallAimed = calcAimedPose(FieldConstants.cargoD.transformBy(new Transform2d(new Translation2d(0.0, 0.2), new Rotation2d())));
            LoggingRamseteCommand driveToSecondBall = new LoggingRamseteCommand(driveTrain, secondBallPathStart, Collections.emptyList(), secondBallAimed, false);

            // Drive to terminal
            Pose2d terminalCargoPosition = FieldConstants.cargoG.transformBy(new Transform2d(new Translation2d(0.5, 0.0), Rotation2d.fromDegrees(180.0)));
            Translation2d terminalCargoApproachPosition = terminalCargoPosition.transformBy(GeomUtil.transformFromTranslation(-0.8, 0.0)).getTranslation();
            LoggingRamseteCommand driveToTerminalBall = new LoggingRamseteCommand(driveTrain, secondBallAimed, Collections.emptyList(), terminalCargoPosition, false);

            
            // Drive to final shot location
            Pose2d finalShotLocation = calcAimedPose(secondBallAimed.transformBy(new Transform2d(new Translation2d(1, -1), new Rotation2d())));
            LoggingRamseteCommand driveToFinalShot = new LoggingRamseteCommand(driveTrain, terminalCargoPosition, Collections.emptyList(), finalShotLocation, true);

            driveTrain.addTrajectoryToDashboard(driveToFirstBall.getTrajectory());
            driveTrain.addTrajectoryToDashboard(driveToSecondBall.getTrajectory());
            InstantCommand resetOdometryCommand = new InstantCommand(() -> driveTrain.resetOdometry(start));

            addCommands(
                resetOdometryCommand,
                // Drive to first ball and shoot preload/first ball. Spin up shooter on the way
                deadline(driveToFirstBall,
                    new IntakeOut(intake), 
                    new IntakeRun(intake),
                    new Index(lowerConveyor, upperConveyor, colorSensor),
                    new SetFlywheelToLimelightShot(shooter, vision)),

                deadline(new ShootBall(upperConveyor, lowerConveyor, colorSensor).withTimeout(1.5), 
                    new IntakeRun(intake),
                    new SetFlywheelToLimelightShot(shooter, vision)),

                // Drive to second ball
                turnTowardsSecondBall.withTimeout(0.5), 
                deadline(driveToSecondBall,  
                    new IntakeRun(intake),
                    new Index(lowerConveyor, upperConveyor, colorSensor)),

                // Shoot second ball
                deadline(new WaitCommand(0.25).andThen(new ShootBall(upperConveyor, lowerConveyor, colorSensor).withTimeout(1)),
                    new SetFlywheelToLimelightShot(shooter, vision),
                    new DriveWithLimelight(driveTrain, vision)),

                // Drive to terminal with slight pause
                deadline(driveToTerminalBall.andThen(new WaitCommand(0.5)), 
                    new IntakeRun(intake), 
                    new Index(lowerConveyor, upperConveyor, colorSensor)),

                // Drive to final shot location
                deadline(
                    driveToFinalShot, 
                    new IntakeRun(intake),
                    new Index(lowerConveyor, upperConveyor, colorSensor),
                    new SetFlywheelToLimelightShot(shooter, vision)),

                // Shoot final ball
                parallel(new ShootBall(upperConveyor, lowerConveyor, colorSensor),
                    new SetFlywheelToLimelightShot(shooter, vision),
                    new DriveWithLimelight(driveTrain, vision))


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
