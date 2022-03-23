// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AutoAim;
import frc.robot.commands.AutoIndex;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.CheckButton;
import frc.robot.commands.ClimberDown;
import frc.robot.commands.ClimberUp;
import frc.robot.commands.DriveByDistance;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.DriveWithLimelight;
import frc.robot.commands.FreeClimb;
import frc.robot.commands.Index;
import frc.robot.commands.IntakeIn;
import frc.robot.commands.IntakeOut;
import frc.robot.commands.IntakeRun;
import frc.robot.commands.Purge;
import frc.robot.commands.ResetDriveTrainEncoders;
import frc.robot.commands.SetFlywheelToFarShot;
import frc.robot.commands.SetFlywheelToFenderShot;
import frc.robot.commands.SetFlywheelToLimelightShot;
import frc.robot.commands.SetFlywheelToLimelightShotTimed;
import frc.robot.commands.SetFlywheelToLowShot;
import frc.robot.commands.ShootBall;
import frc.robot.commands.TurnByAngle;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LowerConveyor;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.UpperConveyor;
import frc.robot.subsystems.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // subsystem declare
  private final DriveTrain driveTrain;
  private final Shooter shooter;
  private final Climber climber;
  private final Intake intake;
  private final LowerConveyor lowerCon;
  private final UpperConveyor upperCon;
  private final Vision vision;

  // controller declare
  public static XboxController driverJoyStick;
  public static XboxController operatorJoyStick;

  // Autonomous chooser
  private final SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    driverJoyStick = new XboxController(Constants.Controller.DriveJoystickNumber);
    operatorJoyStick = new XboxController(Constants.Controller.OperatorJoystickNumber);
    // set values for subsystems
    driveTrain = new DriveTrain();
    shooter = new Shooter();
    climber = new Climber();
    intake = new Intake();
    lowerCon = new LowerConveyor();
    upperCon = new UpperConveyor();
    vision = new Vision();

    // Create autonomous chooser
    autoChooser = new SendableChooser<>();
    autoChooser.setDefaultOption("Three Ball Auto", threeBallAuto());
    autoChooser.addOption("Two Ball Auto", twoBallAuto());
    SmartDashboard.putData(autoChooser);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // Driver Joystick
    driveTrain.setDefaultCommand(new DriveWithJoysticks(driveTrain));

    Button driverAButton = new JoystickButton(driverJoyStick, XboxController.Button.kA.value);
    driverAButton.whenHeld(new ParallelCommandGroup(
        new DriveWithLimelight(driveTrain, vision),
        new SetFlywheelToLimelightShot(shooter, vision)));

    Button driverRightTrigger = new Button(() -> driverJoyStick.getRightTriggerAxis() > 0.5);
    driverRightTrigger.whenHeld(new ShootBall(upperCon, lowerCon, shooter::isShooterAtSpeed));

    Button driverLeftTrigger = new Button(() -> driverJoyStick.getLeftTriggerAxis() > 0.5);
    driverLeftTrigger.whileHeld(new IntakeRun(intake));

    // Operator Joystick
    Button operatorLeftBumper = new JoystickButton(operatorJoyStick, XboxController.Button.kLeftBumper.value);
    operatorLeftBumper.whenHeld(new IntakeOut(intake));
    operatorLeftBumper.whenPressed(new Index(lowerCon, upperCon).andThen(new IntakeIn(intake)));

    Button operatorRightBumper = new JoystickButton(operatorJoyStick, XboxController.Button.kRightBumper.value);
    operatorRightBumper.whenHeld(new IntakeIn(intake));

    Button operatorYButton = new JoystickButton(operatorJoyStick, XboxController.Button.kY.value);
    operatorYButton.whenHeld(new SetFlywheelToFarShot(shooter));

    Button operatorXButton = new JoystickButton(operatorJoyStick, XboxController.Button.kX.value);
    operatorXButton.whenHeld(new SetFlywheelToFenderShot(shooter));

    Button operatorAButton = new JoystickButton(operatorJoyStick, XboxController.Button.kA.value);
    operatorAButton.whenHeld(new SetFlywheelToLowShot(shooter));

    Button operatorRightTrigger = new Button(() -> operatorJoyStick.getRightTriggerAxis() > 0.5);
    operatorRightTrigger.whenHeld(new Purge(intake, lowerCon, upperCon, shooter));

    climber.setDefaultCommand(new FreeClimb(climber));

    Button operatorDPadRight = new Button(() -> operatorJoyStick.getPOV() == 90);
    operatorDPadRight.whenPressed(new InstantCommand(climber::climberStraight));

    Button operatorDPadLeft = new Button(() -> operatorJoyStick.getPOV() == 270);
    operatorDPadLeft.whenPressed(new InstantCommand(climber::climberTilt));

    Button operatorDPadUp = new Button(() -> operatorJoyStick.getPOV() == 0);
    operatorDPadUp.whenPressed(new ClimberUp(climber));

    Button operatorDPadDown = new Button(() -> operatorJoyStick.getPOV() == 180);
    operatorDPadDown.whenPressed(new ClimberDown(climber));

    Button operatorStartButton = new JoystickButton(operatorJoyStick, XboxController.Button.kStart.value);
    operatorStartButton.whenHeld(moveToNextBarCommand()
        .andThen(new WaitCommand(1.5))
        .andThen(moveToNextBarCommand()));
  }

  private Command moveToNextBarCommand() {
    JoystickButton operatorBackButton = new JoystickButton(operatorJoyStick, XboxController.Button.kBack.value);
    return new SequentialCommandGroup(
        new ParallelCommandGroup(new ClimberUp(climber), new InstantCommand(climber::climberTilt)),
        new InstantCommand(climber::climberStraight),
        new ParallelCommandGroup(new WaitCommand(.5), new CheckButton(operatorBackButton)),
        new ClimberDown(climber));
  }

  public Command twoBallAuto() {
    return new SequentialCommandGroup(
      new IntakeOut(intake),
      new ResetDriveTrainEncoders(driveTrain),
      new WaitCommand(0.1),
      new ParallelCommandGroup(
        new Index(lowerCon, upperCon), 
        new DriveByDistance(1, driveTrain)),
      new IntakeIn(intake),
      new ParallelCommandGroup(
        new AutoAim(driveTrain, vision, 0.5),
        new SetFlywheelToLimelightShotTimed(shooter, vision, 0.5)),
      new ParallelCommandGroup(
        new AutoAim(driveTrain, vision, 2),
        new SetFlywheelToLimelightShotTimed(shooter, vision, 2),
        new AutoShoot(upperCon, lowerCon, shooter::isShooterAtSpeed, 2)),
      new ResetDriveTrainEncoders(driveTrain),
      new DriveByDistance(0.3, driveTrain)
    );
  }

  public Command threeBallAuto() {
    return new SequentialCommandGroup(
      new IntakeOut(intake),
      new ResetDriveTrainEncoders(driveTrain),
      new WaitCommand(0.1),
      new ParallelRaceGroup(
        new AutoIndex(lowerCon, upperCon, 3), 
        new IntakeRun(intake),
        new DriveByDistance(1, driveTrain)),
      new ParallelCommandGroup(
        new AutoAim(driveTrain, vision, 0.5),
        new SetFlywheelToLimelightShotTimed(shooter, vision, 0.5)),
      new ParallelCommandGroup(
        new AutoAim(driveTrain, vision, 2),
        new SetFlywheelToLimelightShotTimed(shooter, vision, 2),
        new AutoShoot(upperCon, lowerCon, shooter::isShooterAtSpeed, 2)),
      new ResetDriveTrainEncoders(driveTrain),
      new DriveByDistance(-0.3, driveTrain),
      new ResetDriveTrainEncoders(driveTrain),
      new WaitCommand(.1),
      new TurnByAngle(-108.5, driveTrain),
      new ResetDriveTrainEncoders(driveTrain),
      new WaitCommand(0.1),
      new IntakeOut(intake),
      new ParallelRaceGroup(
        new AutoIndex(lowerCon, upperCon, 3), 
        new IntakeRun(intake),
        new DriveByDistance(3.0, driveTrain)),
      new ResetDriveTrainEncoders(driveTrain),
      new WaitCommand(0.1),
      new TurnByAngle(-58, driveTrain),
      new IntakeIn(intake),
      new ParallelCommandGroup(
        new AutoAim(driveTrain, vision, 0.5),
        new SetFlywheelToLimelightShotTimed(shooter, vision, 0.5)),
      new ParallelCommandGroup(
        new AutoAim(driveTrain, vision, 2),
        new SetFlywheelToLimelightShotTimed(shooter, vision, 2),
        new AutoShoot(upperCon, lowerCon, shooter::isShooterAtSpeed, 2))
    );
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
