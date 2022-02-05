// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveForwardTimed;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.ExtendClimber;
import frc.robot.commands.FreeClimb;
import frc.robot.commands.DriveWithLimelight;
import frc.robot.commands.ToggleIntakeOutIn;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
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
  // private final Climber climber;
  //private final Intake intake;
  //private final Vision vision;
  // command declare
  private final DriveForwardTimed driveForwardTimed;
  private final DriveWithJoysticks driveWithJoysticks;
  // private final FreeClimb freeClimb;
  // private final ExtendClimber extendClimber;
  //private final ToggleIntakeOutIn toggleIntake;
  //private final LimelightAim limelightAim;
  // controller declare
  public static XboxController driverJoyStick;
  public static XboxController operatorJoyStick;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    driverJoyStick = new XboxController(Constants.DriveJoystickNumber);
    operatorJoyStick = new XboxController(Constants.OperatorJoystickNumber);
    // set values for subsystems
    driveTrain = new DriveTrain();
    // climber = new Climber();
    //intake = new Intake();
    //vision = new Vision();
    // Set the default command settings
    driveWithJoysticks = new DriveWithJoysticks(driveTrain);
    driveWithJoysticks.addRequirements(driveTrain);
    driveTrain.setDefaultCommand(driveWithJoysticks);

    // extendClimber = new ExtendClimber(climber);
    // extendClimber.addRequirements(climber);
    // climber.setDefaultCommand(extendClimber);

    // freeClimb = new FreeClimb(climber);
    // freeClimb.addRequirements(climber);
    // climber.setDefaultCommand(freeClimb);

    // set values for commands and set default commands
    driveForwardTimed = new DriveForwardTimed(driveTrain);
    driveForwardTimed.addRequirements(driveTrain);

    //toggleIntake = new ToggleIntakeOutIn(intake);
    //toggleIntake.addRequirements(intake);

    //limelightAim = new LimelightAim(driveTrain, vision);
    //limelightAim.addRequirements(driveTrain, vision);

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
    // JoystickButton toggleIntakeButton_B = new JoystickButton(operatorJoyStick, XboxController.Button.kB.value);
    // toggleIntakeButton_B.whenPressed(toggleIntake);

    JoystickButton limelightAim_A = new JoystickButton(driverJoyStick, XboxController.Button.kA.value);
    // limelightAim_A.whileHeld(limelightAim);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return driveForwardTimed;
  }
}
