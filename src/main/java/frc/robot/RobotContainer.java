// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.lowerCon;
import frc.robot.commands.ClimberDown;
import frc.robot.commands.ClimberUp;
import frc.robot.commands.DriveForwardTimed;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.DriveWithLimelight;
import frc.robot.commands.ExtendClimber;
import frc.robot.commands.FreeClimb;
import frc.robot.commands.IntakeIn;
import frc.robot.commands.IntakeOut;
import frc.robot.commands.IntakeRun;
import frc.robot.commands.IntakeStop;
import frc.robot.commands.LowerConStop;
import frc.robot.commands.LowerConveyorIntake;
import frc.robot.commands.RetractClimber;
import frc.robot.commands.SetFlywheelToFarShot;
import frc.robot.commands.SetFlywheelToFenderShot;
import frc.robot.commands.ShootBall;
import frc.robot.commands.UpperConveyorIntake;
import frc.robot.commands.UpperConveyorStop;
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
  private final LowerConveyor lowCon;
  private final UpperConveyor upperCon;
  private final Vision vision;

  // controller declare
  public static XboxController driverJoyStick;
  public static XboxController operatorJoyStick;

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
    lowCon = new LowerConveyor();
    upperCon = new UpperConveyor();
    vision = new Vision();

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
    driverAButton.whenHeld(new DriveWithLimelight(driveTrain, vision));
    Button driverRightTrigger = new Button(() -> driverJoyStick.getRightTriggerAxis() > 0.5);
    driverRightTrigger.whenHeld(new ShootBall(upperCon, lowCon));
    Button driverRightBumper = new JoystickButton(driverJoyStick, XboxController.Button.kRightBumper.value);
    // driverRightBumper.whenHeld(manualConveyorForward);
    Button driverLeftTrigger = new Button(() -> driverJoyStick.getLeftTriggerAxis() > 0.5);
    driverLeftTrigger.whileHeld(new IntakeRun(intake));
    driverLeftTrigger.whileHeld(new LowerConveyorIntake(lowCon));
    driverLeftTrigger.whileHeld(new UpperConveyorIntake(upperCon));
    //driverLeftTrigger.whenHeld(intakePause);
    

    //Operator Joystick 
    Button operatorLeftBumper = new JoystickButton(operatorJoyStick, XboxController.Button.kLeftBumper.value);
    operatorLeftBumper.whenHeld(new SetFlywheelToFenderShot(shooter));
    Button operatorRightBumper = new JoystickButton(operatorJoyStick , XboxController.Button.kRightBumper.value);
    operatorRightBumper.whenHeld(new SetFlywheelToFarShot(shooter));
    Button operatorYButton = new JoystickButton(operatorJoyStick, XboxController.Button.kY.value);
    operatorYButton.whenHeld(new IntakeIn(intake));
    operatorYButton.whenHeld(new IntakeStop(intake));
    operatorYButton.whenHeld(new LowerConStop(lowCon));
    operatorYButton.whenHeld(new UpperConveyorStop(upperCon));
    Button operatorAButton = new JoystickButton(operatorJoyStick, XboxController.Button.kA.value);
    operatorAButton.whenHeld(new IntakeOut(intake));
    Button operatorBButton = new JoystickButton(operatorJoyStick, XboxController.Button.kB.value);
    // operatorBButton.whenHeld(purge);
    //climber.setDefaultCommand(new FreeClimb(climber));

    Button operatorDPadRight = new Button(() -> operatorJoyStick.getPOV() == 90);
    operatorDPadRight.whenHeld(new ExtendClimber(climber));
    Button operatorDPadLeft = new Button(() -> operatorJoyStick.getPOV() == 270);
    operatorDPadLeft.whenHeld(new RetractClimber(climber));
    Button operatorDPadUp = new Button(() -> operatorJoyStick.getPOV() == 0);
    operatorDPadUp.whenHeld(new ClimberUp(climber));
    Button operatorDPadDown = new Button(() -> operatorJoyStick.getPOV() == 180);
    operatorDPadDown.whenHeld(new ClimberDown(climber));
    Button operatorStartButton = new JoystickButton(operatorJoyStick, XboxController.Button.kStart.value);
    // operatorStartButton.whenHeld(autoClimb);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new DriveForwardTimed(driveTrain);
  }
}
