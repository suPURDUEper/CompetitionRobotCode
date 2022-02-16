// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class DriveTrain {
        public static final double BoostActive = 1.0;
        public static final double BoostInactive = 0.7;
        public static final double DriveTrainCurve = 0.15;
        public static final double BaseVelocity = 0.14;
        public static final double DriveTrainSpeed = 0.7;
        public static final int LEFT_FRONT_CAN_ID = 1;
        public static final int LEFT_BACK_CAN_ID = 2;
        public static final int RIGHT_FRONT_CAN_ID = 3;
        public static final int RIGHT_BACK_CAN_ID = 4;
        public static final double TRACK_WIDTH_INCHES = 12;
        public static final double WHEEL_DIAMETER_INCHES = 4;
        public static final int ENCODER_RESOLUTION = 42;
        public static final double GEARBOX_RATIO = 8.42;
        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or theoretically
        // for *your* robot's drive.
        // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
        // values for your robot.
        public static final double ksVolts = 0.22;
        public static final double kvVoltSecondsPerMeter = 1.98;
        public static final double kaVoltSecondsSquaredPerMeter = 0.2;

        // Example value only - as above, this must be tuned for your drive!
        public static final double kPDriveVel = 8.5;

        public static final double kTrackwidthMeters = 0.69;
        public static final double kWheelRadiusMeters = 0.0508;

        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }

    public static class Climber {
        public static final int leftClimbMotor = 8;
        public static final int rightClimbMotor = 9;
    }

    public static class lowerCon {
        public static final int LowConBreakBeam = 0;
        public static final int PooperBreakBeam = 1;
        public static final int LowConMotor = 5;
        public static final int PooperMotor = 6;
    }

    public static class Shooter {
        public static final int HOOD_SERVO_CHANNEL = 1;
        public static final int FlywheelSpeed = 5700;
        public static final double FLYWHEEL_KD = 0;
        public static final double FLYWHEEL_KI = 0;
        public static final double FLYWHEEL_KF = 1023.0/21777.0; // 1023 is max talon internal output units, 20660 is talon max internal velocity sensor units
        public static final double FLYWHEEL_KP = 1023.0/21777.0;
        public static final int PID_LOOP_INDEX = 0;
        public static final int SHOOTER_CURRENT_LIMIT_AMPS = 40;
        public static final int LEFT_FLYWHEEL_CAN_ID = 0;
        public static final int RIGHT_FLYWHEEL_CAN_ID = 1;
        public static final int ACCELERATOR_CAN_ID = 2;
        public static final int HOOD_SOLENOID_FWD_ID = 10; // TODO: Change
        public static final int HOOD_SOLENOID_REV_ID = 11; // TODO: Change
    }

    public static class UpperCon {
        public static final int UpperConMotor = 7;
        public static final int UpperConBreakBeam = 2;
    }

    public static class Intake {
        public static final int INDEXER_MOTOR_ID = 0;
        public static final int INTAKE_MOTOR_TALON_ID = 2;
        public static final int INTAKE_CURRENT_LIMIT_AMPS = 40;
    }

    public static class Autonomous {
        public static final double DriveForwardTime = 3;
        public static final double AutonomousSpeed = 0.4;
    }

    public static class Controller {
        public static final int XboxLeftYAxis = 1;
        public static final int XboxRightXAxis = 4;
        public static final int XboxRightYAxis = 5;
        public static final int DriveJoystickNumber = 0;
        public static final int OperatorJoystickNumber = 1;
        public static final double DeadZone = 0.05;
    }

    public static class Talon {
        public static final double DEFAULT_DEADBAND = 0.04;
    }
}
