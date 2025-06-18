package frc.robot.subsystems.climber;

import static frc.robot.subsystems.climber.ClimberConstants.*;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Servo;

public class ClimberIOReal implements ClimberIO {
  private final Servo servo;
  private boolean isServoUnlocked;
  private final SparkFlex motor;
  private final SparkLimitSwitch reverseLimitSwitch;
  private final RelativeEncoder encoder;

  public ClimberIOReal() {
    servo = new Servo(kServoChannel);
    isServoUnlocked = true;
    setServo(true);
    motor = new SparkFlex(kCanID, MotorType.kBrushless);
    encoder = motor.getEncoder();

    // Configure drive motor
    var motorConfig = new SparkFlexConfig();
    motorConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(kCurrentLimit)
        .voltageCompensation(12.0);
    motorConfig
        .encoder
        .positionConversionFactor(kEncoderPositionFactor)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    motorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    motorConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (1000.0 / 100.0))
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(motor, 5, () -> encoder.setPosition(0.0));
    reverseLimitSwitch = motor.getReverseLimitSwitch();
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.servoUnlocked = isServoUnlocked;
    inputs.connected = motor.getFirmwareVersion() != 0;
    inputs.appliedVolts = motor.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.currentAmps = motor.getOutputCurrent();
    inputs.positionRadians = encoder.getPosition();
    inputs.rpm = encoder.getVelocity();

    if (reverseLimitSwitch.isPressed()) {
      encoder.setPosition(0);
    }
    if (inputs.positionRadians >= kExtendedRadians) {
      inputs.forwardLimitReached = true;
    } else {
      inputs.forwardLimitReached = false;
    }
  }

  @Override
  public void setServo(boolean isUnlocked) {
    if (isUnlocked) {
      servo.setPosition(kServoReleasePosition);
    } else {
      servo.setPosition(kServoBrakePosition);
    }
    isServoUnlocked = isUnlocked;
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }
}
