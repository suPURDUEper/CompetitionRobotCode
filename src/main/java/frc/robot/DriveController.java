package frc.robot;

public class DriveController {
    /**
     * Takes the left y axis of the controller and converts the input to a throttle
     * output
     * for drivetrain
     * @return value between -1.0 and 1.0
     */
    public static double getThrottleMap(double input, double speed) {
        double driveTrainSpeed = 0;
        if (Constants.Controller.DeadZone * -1 < input && input < Constants.Controller.DeadZone) {
            driveTrainSpeed = 0;
        } else {
            if (input > Constants.Controller.DeadZone) {
                driveTrainSpeed = speed
                        * (Constants.DriveTrain.DriveTrainCurve * (1 - Constants.DriveTrain.BaseVelocity)
                                * input * -1
                                + (1 - Constants.DriveTrain.DriveTrainCurve)
                                        * (1 - Constants.DriveTrain.BaseVelocity) * Math.pow(input * -1, 5))
                        + Constants.DriveTrain.BaseVelocity;
            }
            if (input < Constants.Controller.DeadZone * -1) {
                driveTrainSpeed = speed
                        * (Constants.DriveTrain.DriveTrainCurve * (1 - Constants.DriveTrain.BaseVelocity)
                                * input * -1
                                + (1 - Constants.DriveTrain.DriveTrainCurve)
                                        * (1 - Constants.DriveTrain.BaseVelocity) * Math.pow(input * -1, 5))
                        - Constants.DriveTrain.BaseVelocity;
            }
        }
        return driveTrainSpeed;
    }

    /**
     * Takes x axis input and returns a turn command output
     * @return value between -1.0 and 1.0
     */
    public static double getTurnMap(double input, double speed) {
        //double input = Controller.getRawAxis(Constants.Controller.Xboxinput);
        double DriveTrainTurn = 0;
        if (Constants.Controller.DeadZone * -1 < input && input < Constants.Controller.DeadZone) {
          DriveTrainTurn = 0;
        } else {
          if (input > Constants.Controller.DeadZone) {
            DriveTrainTurn = speed * (Constants.DriveTrain.DriveTrainCurve * (1 - Constants.DriveTrain.BaseVelocity)
                * input
                + (1 - Constants.DriveTrain.DriveTrainCurve) * (1 - Constants.DriveTrain.BaseVelocity)
                    * Math.pow(input, 5))
                + Constants.DriveTrain.BaseVelocity;
          }
          if (input < Constants.Controller.DeadZone * -1) {
            DriveTrainTurn = speed * 
            (Constants.DriveTrain.DriveTrainCurve * (1 - Constants.DriveTrain.BaseVelocity) * input)
                + (1 - Constants.DriveTrain.DriveTrainCurve)
                    * (1 - Constants.DriveTrain.BaseVelocity) * Math.pow(input, 5)
                - Constants.DriveTrain.BaseVelocity;
          }
        }
        return DriveTrainTurn;
    }
}
