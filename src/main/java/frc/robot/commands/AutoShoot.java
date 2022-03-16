package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LowerConveyor;
import frc.robot.subsystems.UpperConveyor;

public class AutoShoot extends CommandBase {
  private final UpperConveyor upperConveyor;
  private final LowerConveyor lowerConveyor;
  private Supplier<Boolean> safeToFire;
  private Timer timer = new Timer();
  private double time;
  /** Creates a new AutoShoot. */
  public AutoShoot(UpperConveyor mUpperCon, LowerConveyor mLowCon, Supplier<Boolean> safeToFire, double timeForShot) {
    // Use addRequirements() here to declare subsystem dependencies.
    upperConveyor = mUpperCon;
    lowerConveyor = mLowCon;
    this.safeToFire = safeToFire;
    time = timeForShot;
    addRequirements(mUpperCon, mLowCon);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (safeToFire.get()) {
      lowerConveyor.setLowerConveyorPercentOutput(0.7);
      lowerConveyor.setPooperPercentOutput(0.7);
      upperConveyor.setPercentOutput(1);
    } else {
      lowerConveyor.setLowerConveyorPercentOutput(0);
      lowerConveyor.setPooperPercentOutput(0);
      upperConveyor.setPercentOutput(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    lowerConveyor.setLowerConveyorPercentOutput(0);
    lowerConveyor.setPooperPercentOutput(0);
    upperConveyor.setPercentOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (timer.get() > time);
  }
}
