package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveTrain;

public class TurnByAngle extends PIDCommand {
  /**
   * Turns to robot to the specified angle.
   *
   * @param targetAngleDegrees The angle to turn to
   * @param drive The drive subsystem to use
   */
  public TurnByAngle(double targetAngleDegrees, DriveTrain drive) {
    super(
        new PIDController(0.007, 0, 0),
        // Close loop on heading
        drive::getHeading,
        // Set reference to target
        targetAngleDegrees + drive.getHeading(),
        // Pipe output to turn robot
        output -> drive.arcadeDrive(0, output + Math.copySign(0.3, output)),
        // Require the drive
        drive);

    // Set the controller to be continuous (because it is an angle controller)
    getController().enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController().setTolerance(2, 0.05);
  }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return getController().atSetpoint();
  }
}
