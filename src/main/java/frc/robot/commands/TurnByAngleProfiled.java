package frc.robot.commands;

import static frc.robot.Constants.DriveTrain.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.DriveTrain;

/** A command that will turn the robot by a specified angle using a motion profile. */


public class TurnByAngleProfiled extends CommandBase {
  private ProfiledPIDController controller;
  private DriveTrain drive; 
  double lastSpeed = 0;
  double lastTime = Timer.getFPGATimestamp();
  double targetAngleDegrees;

  /**
   * Turns to robot by a specified angle using a motion profile.
   *
   * @param angle The angle to turn by in degrees
   * @param drive The drive subsystem to use
   */
  public TurnByAngleProfiled(double angle, DriveTrain drive) {
    controller = new ProfiledPIDController(
      0.015,  //kP
      0,      //kI
      0,      //kD
      new TrapezoidProfile.Constraints(
          DRIVE_ANGULAR_VELOCITY_MAX,
          DRIVE_ANGULAR_ACCELERATION_MAX
      )
    );
    this.targetAngleDegrees = drive.getHeading() + angle;
    this.drive = drive;
    addRequirements(drive);
    // Set the controller to be continuous (because it is an angle controller)
    controller.enableContinuousInput(0, 360);
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    controller.setTolerance(
      tangentialToRotational(DRIVE_ANGULAR_POSITION_MAX_ERR), 
      tangentialToRotational(DRIVE_ANGULAR_VELOCTIY_MAX_ERR)
    );
  }

  @Override
  public void initialize() {
    controller.reset(0);
    controller.setGoal(targetAngleDegrees);
  }

  @Override
  public void execute() {
    double pidOutput = controller.calculate(drive.getHeading());
    double feedforward = getFeedforward(controller.getSetpoint());
    double voltage = pidOutput + feedforward;
    drive.setDriveMotorVoltage(-voltage, voltage);
  }

  @Override
  public void end(boolean interrupted) {
    drive.setDriveMotorVoltage(0, 0);
  }

  private double getFeedforward(State setpoint) {
    // Setpoint is in units of degrees and degrees/s, so we need to convert to
    // // wheel velocities for these gains to make sense.
    // double acceleration = (controller.getSetpoint().velocity - lastSpeed) / (Timer.getFPGATimestamp() - lastTime);
    // return DRIVE_LINEAR_FF.calculate(
    //   rotationalToTangential(setpoint.velocity), 
    //   rotationalToTangential(acceleration)
    // );
    return 0.3;
  }

  private double tangentialToRotational(double tangentialMeters) {
    // Left wheel speed is negative, because wpilib code assumes that CCW is positive 
    DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds(-tangentialMeters, tangentialMeters);
    ChassisSpeeds chassisSpeeds = kDriveKinematics.toChassisSpeeds(wheelSpeeds);
    return Math.toDegrees(chassisSpeeds.omegaRadiansPerSecond);
  }

  private double rotationalToTangential(double degrees) {
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0,0,Math.toRadians(degrees));
    DifferentialDriveWheelSpeeds wheelSpeeds = kDriveKinematics.toWheelSpeeds(chassisSpeeds);
    return wheelSpeeds.leftMetersPerSecond;
  }
}