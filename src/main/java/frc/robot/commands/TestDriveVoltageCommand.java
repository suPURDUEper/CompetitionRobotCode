package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class TestDriveVoltageCommand extends CommandBase {

    /** Creates a new UpperConveyorIntake. */
    private final DriveTrain driveTrain;

    public TestDriveVoltageCommand(DriveTrain driveTrain) {
        this.driveTrain = driveTrain;
        addRequirements(driveTrain);
        // Use addRequirements() here to declare subsystem dependencies.
    }
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        driveTrain.setDriveMotorVoltage(4, 4);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        driveTrain.setDriveMotorVoltage(0, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
    return false;
    }
    
}
