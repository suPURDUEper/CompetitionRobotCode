// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

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
    public static int TALON_TIMEOUT = 30;
    public static class DriveTrain {
        public static final double BoostActive = 1.0;
        public static final double BoostInactive = 0.7;
        public static final double DriveTrainCurve = 0.15;
        public static final double BaseVelocity = 0.14;
        public static final double DriveTrainSpeed = 0.7;
        public static final double WHEEL_DIAMETER_INCHES = 4.0;
        public static final int ENCODER_RESOLUTION = 42;
        public static final double GEARBOX_RATIO = 8.45;

        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(WHEEL_DIAMETER_INCHES / 2.0);

        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
        public static final int DRIVE_LEFT_FRONT_CAN_ID = 3;
        public static final int DRIVE_LEFT_BACK_CAN_ID = 4;
        public static final int DRIVE_RIGHT_FRONT_CAN_ID = 1;
        public static final int DRIVE_RIGHT_BACK_CAN_ID = 2;
        public static final double DRIVE_LINEAR_KS = 0.27122;
        public static final double DRIVE_LINEAR_KV = 3.361;
        public static final double DRIVE_LINEAR_KA = 0.60179;
        public static final SimpleMotorFeedforward DRIVE_LINEAR_FF = new SimpleMotorFeedforward(DRIVE_LINEAR_KS, DRIVE_LINEAR_KV, DRIVE_LINEAR_KA);
        public static final double DRIVE_LINEAR_VELOCITY_KP = 0.32029;
        public static final double DRIVE_LINAER_VELOCTIY_MAX_ERR = 0.87241;
        public static final double DRIVE_MAX_CONTROL_EFFORT = 7;
        public static final double DRIVE_LINEAR_POSITION_KP = 139.93;
        public static final double DRIVE_LINEAR_POSITION_KD = 11.657;
        public static final double DRIVE_LINEAR_POSITION_MAX_ERR = 0.03997;
        public static final double DRIVE_ANGULAR_KS = 0.7092;
        public static final double DRIVE_ANGULAR_KV = 3.5767;
        public static final double DRIVE_ANGULAR_KA = 0.43016;
        public static final SimpleMotorFeedforward DRIVE_ANGULAR_FF = new SimpleMotorFeedforward(DRIVE_ANGULAR_KS, DRIVE_ANGULAR_KV, DRIVE_ANGULAR_KA);
        public static final double TRACK_WIDTH_METERS = 0.60389;
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(TRACK_WIDTH_METERS);
        public static final double DRIVE_ANGULAR_POSITION_KP = 132.87;
        public static final double DRIVE_ANGULAR_POSITION_KD = 9.6808;
        public static final double DRIVE_ANGULAR_POSITION_MAX_ERR = 0.03997;
        public static final double DRIVE_ANGULAR_VELOCITY_KP = 0.000068388;
        public static final double DRIVE_ANGULAR_VELOCTIY_MAX_ERR = 0.78919;
        public static final double DRIVE_ANGULAR_VELOCITY_MAX = 360; // deg/s
        public static final double DRIVE_ANGULAR_ACCELERATION_MAX = 720; // deg/s^2





    }

    public static class Climber {
        public static final int LEFT_CLIMB_MOTOR_CAD_ID = 5;
        public static final int RIGHT_CLIMB_MOTOR_CAN_ID = 4;
        public static final double CLIMB_SYNC_KF = 0;
        public static final double CLIMB_SYNC_KP = 0.00007;
        public static final double CLIMB_SYNC_KI = 0;
        public static final double CLIMB_SYNC_KD = 0;
        public static final double CLIMB_LEFT_MAX_HEIGHT = 358000;
        public static final double CLIMB_RIGHT_MAX_HEIGHT = 366000;
        public static final double LEFT_CLIMB_EXTEND_HEIGHT = 0;
        public static final int CLIMB_CURRENT_LIMIT = 60;
    }

    public static class lowerCon {
        public static final int LowConBreakBeam = 0;
        public static final int PooperBreakBeam = 1;
        public static final int LowConMotor = 6;
        public static final int PooperMotor = 7;
    }

    public static class Shooter {
        public static final int HOOD_SOLENOID_FWD_ID = 2;
        public static final int HOOD_SOLENOID_REV_ID = 3;
        public static final int HOOD_SERVO_CHANNEL = 1;
        public static final int FlywheelSpeed = 5700;
        public static final double FLYWHEEL_KD = 0;
        public static final double FLYWHEEL_KI = 0;
        public static final double FLYWHEEL_KF = 1023.0/21777.0; // 1023 is max talon internal output units, 20660 is talon max internal velocity sensor units
        public static final double FLYWHEEL_KP = 1023.0/21777.0;
        public static final int PID_LOOP_INDEX = 0;
        public static final int SHOOTER_CURRENT_LIMIT = 40;
        public static final int LEFT_FLYWHEEL_CAN_ID = 1;
        public static final int RIGHT_FLYWHEEL_CAN_ID = 2;
        public static final int ACCELERATOR_CAN_ID = 3;
        public static final int SHOOTER_CURRENT_LIMIT_AMPS = 40;
        public static final int SHOOTER_FENDER_SHOT_RPM = 2250;
        public static final int SHOOTER_FAR_SHOT_RPM = 3750;
    }

    public static class UpperCon {
        public static final int UpperConMotor = 8;
        public static final int UpperConBreakBeam = 2;
        public static final int MidConBreakBeam = 3; 
    }

    public static class Intake {
        public static final int INDEXER_MOTOR_ID = 5;
        public static final int INTAKE_MOTOR_TALON_ID = 0;
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
