// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
        public static final double BoostActive = 0.9;
        public static final double BoostInactive = 0.7;
        public static final double DriveTrainCurve = 0.15;
        public static final double BaseVelocity = 0.14;
        public static final double DriveTrainSpeed = 0.7;
        public static final int LeftFront = 1;
        public static final int LeftBack = 2;
        public static final int RightFront = 3;
        public static final int RightBack = 4;
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
        public static final double FlywheelkD = 1;
        public static final double FlywheelkI = 0;
        public static final double FlywheelkF = 0.05;
        public static final double FlywheelkP = 4;
        public static int kPIDLoopIdx = 0;
        public static int kTimeoutMs = 30;
        public static final int LeftFlywheel = 0;
        public static final int RightFlywheel = 1;
    }

    public static class UpperCon {
        public static final int UpperConMotor = 7;
        public static final int UpperConBreakBeam = 2;
    }

    public static class Intake {
        public static final int IntakeMotor = 0;
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
}
