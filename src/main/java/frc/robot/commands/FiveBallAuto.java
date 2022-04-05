package frc.robot.commands;

import java.util.Collections;
import java.util.List;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer.AutoPosition;
import frc.robot.subsystems.*;
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
            LoggingRamseteCommand pathToFirstBall = new LoggingRamseteCommand(driveTrain, start, Collections.emptyList(), firstBallAimed);

            // Turn away from wall
            TurnByAngle turnToBall2 = new TurnByAngle(-120, driveTrain);

            // Drive to second ball
            Pose2d startToDriveTo2ndBall = firstBallAimed.transformBy(new Transform2d(new Translation2d(), new Rotation2d(-120)));
            Pose2d secondBallAimed = calcAimedPose(FieldConstants.cargoD.transformBy(new Transform2d(new Translation2d(-0.2, 0.2), new Rotation2d())));
            LoggingRamseteCommand pathToSecondBall = new LoggingRamseteCommand(driveTrain, startToDriveTo2ndBall, Collections.emptyList(), secondBallAimed);

            // Drive to terminal
            Pose2d terminalCargoPosition = FieldConstants.cargoG.transformBy(new Transform2d(new Translation2d(0.5, 0.0), Rotation2d.fromDegrees(180.0)));
            Translation2d terminalCargoApproachPosition = terminalCargoPosition.transformBy(GeomUtil.transformFromTranslation(-0.8, 0.0)).getTranslation();
            LoggingRamseteCommand driveToTerminalBall = new LoggingRamseteCommand(driveTrain, secondBallAimed, Collections.emptyList(), terminalCargoPosition);



            driveTrain.addTrajectoryToDashboard(pathToFirstBall.getTrajectory());
            driveTrain.addTrajectoryToDashboard(pathToSecondBall.getTrajectory());
            InstantCommand resetOdometryCommand = new InstantCommand(() -> driveTrain.resetOdometry(start));

            addCommands(
                resetOdometryCommand,
                new IntakeOut(intake),
                deadline(pathToFirstBall, 
                    new IntakeRun(intake),
                    new IntakeOut(intake), 
                    new Index(lowerConveyor, upperConveyor, colorSensor, this::falseSupplier)),
                race(
                    new SetFlywheelToLimelightShot(shooter, vision), 
                    new WaitCommand(0.25).andThen(new ShootBall(upperConveyor, lowerConveyor, this::trueSupplier).withTimeout(1.5))),
                turnToBall2.withTimeout(0.5), 
                deadline(pathToSecondBall, 
                    new IntakeRun(intake),
                    new IntakeOut(intake), 
                    new Index(lowerConveyor, upperConveyor, colorSensor, this::falseSupplier)),
                race(
                    new SetFlywheelToLimelightShot(shooter, vision), 
                    new WaitCommand(0.25).andThen(new ShootBall(upperConveyor, lowerConveyor, this::trueSupplier).withTimeout(.75))),
                deadline(driveToTerminalBall, 
                    new IntakeRun(intake),
                    new IntakeOut(intake), 
                    new Index(lowerConveyor, upperConveyor, colorSensor, this::falseSupplier))
            );
            addRequirements(driveTrain);

    }

    public boolean falseSupplier() {
        return false;
    }

    public boolean trueSupplier() {
        return true;
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
