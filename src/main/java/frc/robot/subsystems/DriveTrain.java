// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import static frc.robot.Constants.DriveTrain.*;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
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
  private final Encoder leftFrontMockEncoder = new Encoder(3, 4);
  private final Encoder rightFrontMockEncoder = new Encoder(5, 6);
  private final EncoderSim m_leftEncoderSim = new EncoderSim(leftFrontMockEncoder);
  private final EncoderSim m_rightEncoderSim = new EncoderSim(rightFrontMockEncoder);
  private final Field2d fieldDashboardWidget = new Field2d();
  private final DifferentialDrivetrainSim m_drivetrainSimulator = createDrivetrainSim();
  
  public DriveTrain() {
    leftFront = new CANSparkMax(LeftFront, MotorType.kBrushless);
    leftFront.setInverted(true);
    leftBack = new CANSparkMax(LeftBack, MotorType.kBrushless);
    rightFront = new CANSparkMax(RightFront, MotorType.kBrushless);
    rightFront.setInverted(false);
    rightBack = new CANSparkMax(RightBack, MotorType.kBrushless);
    
    leftBack.follow(leftFront);
    rightBack.follow(rightFront);
    drive = new DifferentialDrive(leftFront, rightFront);

    // Setup odometry
    double metersPerPulse = Units.inchesToMeters(Math.PI * WHEEL_DIAMETER_INCHES) / (double) ENCODER_RESOLUTION;
    leftFrontMockEncoder.setDistancePerPulse(metersPerPulse);
    rightFrontMockEncoder.setDistancePerPulse(metersPerPulse);
    leftFront.getEncoder().setPositionConversionFactor(metersPerPulse);
    rightFront.getEncoder().setPositionConversionFactor(metersPerPulse);
    gyro = new AHRS(SerialPort.Port.kMXP);
    odometry = new DifferentialDriveOdometry(gyro.getRotation2d());
    SmartDashboard.putData("Field", fieldDashboardWidget);
  }

  private DifferentialDrivetrainSim createDrivetrainSim() {
    // MOI estimation -- note that I = m r^2 for point masses
    var batteryMoi = 12.5 / 2.2 * Math.pow(Units.inchesToMeters(10), 2);
    var gearboxMoi =
        (2.8 /* CIM motor */ * 2 / 2.2 + 2.0 /* Toughbox Mini- ish */)
            * Math.pow(Units.inchesToMeters(26.0 / 2.0), 2);
    return new DifferentialDrivetrainSim(
      DCMotor.getNEO(2),
      GEARBOX_RATIO,
      batteryMoi + gearboxMoi,
      Units.lbsToKilograms(125 + 12 + 10),
      Units.inchesToMeters(WHEEL_DIAMETER_INCHES / 2),
      Units.inchesToMeters(25),
      null);
  }

  @Override
  public void periodic() {
    if (RobotBase.isReal()) {
      odometry.update(gyro.getRotation2d(), leftFront.getEncoder().getPosition(), rightFront.getEncoder().getPosition());
    } else {
      odometry.update(gyro.getRotation2d(), leftFrontMockEncoder.getDistance(), rightFrontMockEncoder.getDistance());
    }
    fieldDashboardWidget.setRobotPose(odometry.getPoseMeters());
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
    angle.set(-m_drivetrainSimulator.getHeading().getDegrees());
  }
}
