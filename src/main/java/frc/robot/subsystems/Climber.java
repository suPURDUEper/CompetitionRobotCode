package frc.robot.subsystems;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  private final DoubleSolenoid climberSolenoid;
  private final TalonFX masterClimbMotor;
  private final TalonFX followerClimbMotor;
  /**
   * Creates a new climber object.
   */
  public Climber() {
    // double acting solenoid
    // double acting cylinder

    masterClimbMotor = new TalonFX(Constants.Climber.leftClimbMotor);
    masterClimbMotor.setInverted(false);

    followerClimbMotor = new TalonFX(Constants.Climber.rightClimbMotor);
    // any command give to the master climb motor is now also passed to the follower
    followerClimbMotor.follow(followerClimbMotor);
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
   * @param controller the controller object for the operator
   */
  public void climb(XboxController controller) {
    // getRightY() is a new function introduced in the 2022 wpilib
    // removes the need for axis id
    masterClimbMotor.set(ControlMode.PercentOutput, controller.getRightY());
  }
  public void controlSolenoid(XboxController controller) {
    Debouncer mDebouncer = new Debouncer(0.1, DebounceType.kBoth);

    // calculates if the button is true or not
    if (mDebouncer.calculate(controller.getXButton())) {
      climberSolenoid.toggle();
    }
  }
}
