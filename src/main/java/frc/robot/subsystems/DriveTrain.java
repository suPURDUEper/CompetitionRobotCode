// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import static frc.robot.Constants.DriveTrain.*;

import java.util.Collections;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {

  private final CANSparkMax leftFront;
  private final CANSparkMax rightFront;
  private final CANSparkMax leftBack;
  private final CANSparkMax rightBack;
  private final AHRS gyro;
  private final DifferentialDrive drive;
  private final DifferentialDriveOdometry odometry;


  // Simulation classes help us simulate our robot
  private final Encoder leftEncoder = new Encoder(1, 0);
  private final Encoder rightEncoder = new Encoder(4, 5);
  private final EncoderSim m_leftEncoderSim = new EncoderSim(leftEncoder);
  private final EncoderSim m_rightEncoderSim = new EncoderSim(rightEncoder);
  private final Field2d fieldDashboardWidget = new Field2d();
  private final DifferentialDrivetrainSim m_drivetrainSimulator = createDrivetrainSim();

  private static final double DRIVE_METERS_PER_PULSE_BUILTIN = 1 / 22.756;
  private static final double DRIVE_METERS_PER_PULSE_EXTERNAL = 1;

  
  public DriveTrain() {
    leftFront = new CANSparkMax(DRIVE_LEFT_FRONT_CAN_ID, MotorType.kBrushless);
    leftFront.setInverted(false);
    leftBack = new CANSparkMax(DRIVE_LEFT_BACK_CAN_ID, MotorType.kBrushless);
    rightFront = new CANSparkMax(DRIVE_RIGHT_FRONT_CAN_ID, MotorType.kBrushless);
    rightFront.setInverted(true);
    rightBack = new CANSparkMax(DRIVE_RIGHT_BACK_CAN_ID, MotorType.kBrushless);
    
    leftBack.follow(leftFront);
    rightBack.follow(rightFront);
    drive = new DifferentialDrive(leftFront, rightFront);

    // Setup odometry
    //double metersPerRevolution = Units.inchesToMeters(Math.PI * WHEEL_DIAMETER_INCHES);
    leftFront.getEncoder().setPositionConversionFactor(DRIVE_METERS_PER_PULSE_BUILTIN);
    rightFront.getEncoder().setPositionConversionFactor(DRIVE_METERS_PER_PULSE_BUILTIN);
    leftEncoder.setDistancePerPulse(Math.PI * WHEEL_DIAMETER_METERS / ENCODER_RESOLUTION);
    rightEncoder.setDistancePerPulse(Math.PI * WHEEL_DIAMETER_METERS / ENCODER_RESOLUTION);
    gyro = new AHRS();
    odometry = new DifferentialDriveOdometry(new Rotation2d());
    SmartDashboard.putData("Field", fieldDashboardWidget);
  }

  private DifferentialDrivetrainSim createDrivetrainSim() {
    return new DifferentialDrivetrainSim(
      LinearSystemId.identifyDrivetrainSystem(DRIVE_LINEAR_KV, DRIVE_LINEAR_KA, DRIVE_ANGULAR_KV, DRIVE_ANGULAR_KA),
      DCMotor.getNEO(2),
      GEARBOX_RATIO,
      TRACK_WIDTH_METERS,
      WHEEL_DIAMETER_METERS/2,
      null
    );
  }

  @Override
  public void periodic() {
    odometry.update(getGyroRotation2D(), leftEncoder.getDistance(), rightEncoder.getDistance());
    fieldDashboardWidget.setRobotPose(odometry.getPoseMeters());
    SmartDashboard.putNumber("Left Meters", getLeftDriveMeters());
    SmartDashboard.putNumber("Right Meters", getRightDriveMeters());
    SmartDashboard.putNumber("Left Current", leftFront.getOutputCurrent());
    SmartDashboard.putNumber("Right Current", rightFront.getOutputCurrent());
  }

  public void addTrajectoryToDashboard(Trajectory trajectory) {
    fieldDashboardWidget.getObject("traj").setTrajectory(trajectory);
  }

  public void removeTrajectoryFromDashboard() {
    fieldDashboardWidget.getObject("traj").setPoses(Collections.emptyList());

  }

  public void driveWithJoysticks(double throttle, double turn) {
    // arcadeDrive(throttle, turn);
    // drive.arcadeDrive(xSpeed, zRotation);
    drive.curvatureDrive(throttle, turn, Math.abs(throttle) < 0.2);
  }

  public void driveForward(double speed) {
    drive.tankDrive(speed, speed);
  }

  public void arcadeDrive(double throttle, double turn) {
    // false as 3rd is to disable wpilibs squaring of the inputs
    drive.arcadeDrive(throttle, turn);
  }

  public void stop() {
    drive.stopMotor();
  }

  public void setDriveMotorVoltage(double leftVoltage, double rightVoltage) {
    leftFront.setVoltage(leftVoltage);
    rightFront.setVoltage(rightVoltage);
    drive.feedWatchdog();
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, getGyroRotation2D());
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    leftFront.getEncoder().setPosition(0.0);
    rightFront.getEncoder().setPosition(0.0);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (getLeftDriveMeters() + getRightDriveMeters()) / 2.0;
  }

  public double getLeftDriveMeters() {
    return leftEncoder.getDistance();
  }

  public double getRightDriveMeters() {
    return rightEncoder.getDistance();
  }

  public Rotation2d getGyroRotation2D() {
    return Rotation2d.fromDegrees(gyro.getAngle());
  }


  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return getGyroRotation2D().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    // AHRS::getRate is clockwise positive, we want everything to be counterclockwise positive
    // but we are mounted upsidedown, so it works out. 
    return gyro.getRate();
  }

  /** Update our simulation. This should be run every robot loop in simulation. */
  @Override
  public void simulationPeriodic() {
    // To update our simulation, we set motor voltage inputs, update the
    // simulation, and write the simulated positions and velocities to our
    // simulated encoder and gyro. We negate the right side so that positive
    // voltages make the right side move forward.
    if (DriverStation.isTeleop()) {
      m_drivetrainSimulator.setInputs(
        leftFront.get() * RobotController.getInputVoltage(),
        rightFront.get() * RobotController.getInputVoltage());
    } else {
      m_drivetrainSimulator.setInputs(
        leftFront.getAppliedOutput() * RobotController.getInputVoltage(),
        rightFront.getAppliedOutput() * RobotController.getInputVoltage());
    }

    m_drivetrainSimulator.update(0.02);

    m_leftEncoderSim.setDistance(m_drivetrainSimulator.getLeftPositionMeters());
    m_leftEncoderSim.setRate(m_drivetrainSimulator.getLeftVelocityMetersPerSecond());
    m_rightEncoderSim.setDistance(m_drivetrainSimulator.getRightPositionMeters());
    m_rightEncoderSim.setRate(m_drivetrainSimulator.getRightVelocityMetersPerSecond());

    // m_gyroSim.setAngle(-m_drivetrainSimulator.getHeading().getDegrees());
    int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
    angle.set(m_drivetrainSimulator.getHeading().getDegrees());
  }
}
