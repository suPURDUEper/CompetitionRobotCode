package frc.robot.motors;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.SparkMaxAnalogSensor.Mode;

public class CANSparkMaxWrapper extends CANSparkMax {

  public static final int MOTOR_NEO = 0;
  public static final int MOTOR_NEO_550 = 1;

  private CANSparkMaxWrapper(CANSparkMaxWrapper.Builder builder) {
    // Initialize the motor controller
    super(builder.canId, MotorType.kBrushless);
    // Restore factory defaults
    restoreFactoryDefaults();

    // Set appropriate current limit for motors.
    // These limits are just to prevent the motors from
    // burning; you may want lower limits depending on
    // the application
    if (builder.motorType == MOTOR_NEO) {
      setSmartCurrentLimit(50);
    } else if (builder.motorType == MOTOR_NEO_550) {
      setSmartCurrentLimit(20);
    }

    // Set brake or coast mode
    setIdleMode(builder.idleMode);

    if (builder.leader != null) {
      follow(builder.leader, builder.invertFollow);
    }

    // Burn flash so settings are saved in case the controller browns out
    burnFlash();

    // Turn down status frames (this reduces CAN bus utilization)
    // Motor output and faults
    setPeriodicFramePeriod(PeriodicFrame.kStatus0, 1000);

    // Report motor velocity/temperature/voltage/current
    setPeriodicFramePeriod(PeriodicFrame.kStatus1, 1000);

    // Report motor position
    setPeriodicFramePeriod(PeriodicFrame.kStatus2, 1000);

    // Analog sensor
    setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
  }

  @Override
  public SparkMaxAnalogSensor getAnalog(Mode mode) {
    // Turn the status frame period back to default for analog sensor
    setPeriodicFramePeriod(PeriodicFrame.kStatus3, 50);
    return super.getAnalog(mode);
  }

  @Override
  public REVLibError follow(CANSparkMax leader) {
    // Turn the status frame up on the leader, since we want frequent updates to
    // follow their position
    leader.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);
    // Since we never send a set command to this motor, turn the periodc frame down
    setPeriodicFramePeriod(PeriodicFrame.kStatus0, 1000);
    return super.follow(leader);
  }

  @Override
  public RelativeEncoder getEncoder() {
    // Doing closed loop control on the motor, set the appropriate status frames
    // faster
    setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
    setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
    return super.getEncoder();
  }

  public class Builder {
    private int canId;
    private CANSparkMax.IdleMode idleMode;
    private int motorType;
    private CANSparkMax leader;
    private boolean invertFollow;

    public Builder canId(int canId) {
      this.canId = canId;
      return this;
    }

    public Builder idleMode(CANSparkMax.IdleMode idleMode) {
      this.idleMode = idleMode;
      return this;
    }

    public Builder motor(int motorType) {
      this.motorType = motorType;
      return this;
    }

    public Builder follow(CANSparkMax leader, boolean invert) {
      this.leader = leader;
      this.invertFollow = invert;
      return this;
    }

    public CANSparkMaxWrapper build() {
      return new CANSparkMaxWrapper(this);
    }
  }
}
