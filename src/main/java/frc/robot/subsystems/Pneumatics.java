// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pneumatics extends SubsystemBase {
  private DoubleSolenoid climberSolenoid;
  /**
   * Creates a new Pneumatics
   */
  public Pneumatics() {
    climberSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
    // initialize the climber to forward
    // this way the toggle function works
    climberSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * 
   * @param controller the operator controller object
   */
  public void controlSolenoid(XboxController controller) {
    Debouncer mDebouncer = new Debouncer(0.1, DebounceType.kBoth);

    // calculates if the button is true or not
    if (mDebouncer.calculate(controller.getXButton())) {
      climberSolenoid.toggle();
    }
  }
}
