package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * ShuffleboardInfo provides the layout for the shuffle board and some NetworkEntries to be used later by the
 * Limelight Aim as well as the Autonomous Chooser
 */
public class ShuffleboardInfo {
    
    /**
     * The driver tab on the shuffle board
     */
    private final ShuffleboardTab driverTab;

    /**
     * 
     */
    private final NetworkTableEntry mIsTargetValid;

    private final NetworkTableEntry mKpSteer, mKpDrive;

    private static ShuffleboardInfo instance = null;

    private ShuffleboardInfo() {
        driverTab = Shuffleboard.getTab("driver");

        mIsTargetValid = driverTab.add("Valid Target?", false)
            .withPosition(0, 0).withSize(1, 1).withWidget(BuiltInWidgets.kBooleanBox).getEntry();
        
        mKpSteer = Shuffleboard.getTab("Limelight Aim").add("Steering KP", -0.05).getEntry();
        mKpDrive = Shuffleboard.getTab("Limelight Aim").add("Drive KP", -0.05).getEntry();
    }

    public static ShuffleboardInfo getInstance() {
        if (instance == null) {
            instance = new ShuffleboardInfo();
        }

        return instance;
    }

    public NetworkTableEntry getIsTargetValid() {
        return mIsTargetValid;
    }

    public NetworkTableEntry getKpSteer() {
        return mKpSteer;
    }

    public NetworkTableEntry getKpDrive() {
        return mKpDrive;
    }

    public void addAutoChooser(SendableChooser<Command> mAutoChooser) {
        driverTab.add("Autonomous Chooser", mAutoChooser)
        .withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(1,0).withSize(2,1);
    }
}
