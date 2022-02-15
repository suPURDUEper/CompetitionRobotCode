// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {

  private final CANSparkMax leftFront;
  private final CANSparkMax rightFront;
  private final CANSparkMax leftBack;
  private final CANSparkMax rightBack;
  private final AHRS gyro;
  private final DifferentialDrive drive;
  private final DifferentialDriveOdometry odometry;
  public double boost = Constants.DriveTrain.BoostInactive;
  private final Field2d fieldDashboardWidget = new Field2d();

  // Simulation classes help us simulate our robot
  private final AnalogGyroSim m_gyroSim = new AnalogGyroSim(gyro);
  private final EncoderSim m_leftEncoderSim = new EncoderSim(m_leftEncoder);
  private final EncoderSim m_rightEncoderSim = new EncoderSim(m_rightEncoder);
  private final Field2d m_fieldSim = new Field2d();
  private final LinearSystem<N2, N2, N2> m_drivetrainSystem =
      LinearSystemId.identifyDrivetrainSystem(1.98, 0.2, 1.5, 0.3);
  private final DifferentialDrivetrainSim m_drivetrainSimulator =
      new DifferentialDrivetrainSim(
          m_drivetrainSystem, DCMotor.getNEO(2), 12.76, Constants.DriveTrain.kTrackwidthMeters, Constants.DriveTrain.kWheelRadiusMeters, null);


  public DriveTrain() {
    leftFront = new CANSparkMax(Constants.DriveTrain.LeftFront, MotorType.kBrushless);
    leftFront.setInverted(false);
    leftBack = new CANSparkMax(Constants.DriveTrain.LeftBack, MotorType.kBrushless);
    rightFront = new CANSparkMax(Constants.DriveTrain.RightFront, MotorType.kBrushless);
    rightFront.setInverted(true);
    rightBack = new CANSparkMax(Constants.DriveTrain.RightBack, MotorType.kBrushless);
    leftBack.follow(leftFront);
    rightBack.follow(rightFront);
    drive = new DifferentialDrive(leftFront, rightFront);
    gyro = new AHRS(SerialPort.Port.kMXP);
    odometry = new DifferentialDriveOdometry(gyro.getRotation2d());
    SmartDashboard.putData("Field", fieldDashboardWidget);
  }

  @Override
  public void periodic() {
    odometry.update(gyro.getRotation2d(), leftFront.getEncoder().getPosition(), rightFront.getEncoder().getPosition());
    fieldDashboardWidget.setRobotPose(odometry.getPoseMeters());
  }

  public void driveWithJoysticks(double throttle, double turn) {
    arcadeDrive(throttle, turn);
    // drive.arcadeDrive(xSpeed, zRotation);
  }

  public void driveForward(double speed) {
    drive.tankDrive(speed, speed);
  }

  public void arcadeDrive(double throttle, double turn) {
    // false as 3rd is to disable wpilibs squaring of the inputs
    drive.arcadeDrive(throttle, turn, false);
  }

  public void stop() {
    drive.stopMotor();
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
    return new DifferentialDriveWheelSpeeds(leftFront.getEncoder().getVelocity(), rightFront.getEncoder().getVelocity());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, gyro.getRotation2d());
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
    return (leftFront.getEncoder().getPosition() + rightFront.getEncoder().getPosition()) / 2.0;
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
    return gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return gyro.getRate();
  }

  /** Update our simulation. This should be run every robot loop in simulation. */
  public void simulationPeriodic() {
    // To update our simulation, we set motor voltage inputs, update the
    // simulation, and write the simulated positions and velocities to our
    // simulated encoder and gyro. We negate the right side so that positive
    // voltages make the right side move forward.
    m_drivetrainSimulator.setInputs(
        leftFront.get() * RobotController.getInputVoltage(),
        rightFront.get() * RobotController.getInputVoltage());
    m_drivetrainSimulator.update(0.02);

    m_leftEncoderSim.setDistance(m_drivetrainSimulator.getLeftPositionMeters());
    m_leftEncoderSim.setRate(m_drivetrainSimulator.getLeftVelocityMetersPerSecond());
    m_rightEncoderSim.setDistance(m_drivetrainSimulator.getRightPositionMeters());
    m_rightEncoderSim.setRate(m_drivetrainSimulator.getRightVelocityMetersPerSecond());
    // m_gyroSim.setAngle(-m_drivetrainSimulator.getHeading().getDegrees());
    int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
    angle.set(-m_drivetrainSimulator.getHeading().getDegrees());
  }

}
