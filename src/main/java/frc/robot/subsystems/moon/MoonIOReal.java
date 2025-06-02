package frc.robot.subsystems.moon;

import static frc.robot.subsystems.moon.MoonConstants.*;
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

public class MoonIOReal implements MoonIO {
  private final SparkFlex motor;
  private final RelativeEncoder encoder;
  private final SparkLimitSwitch reverseLimitSwitch;

  public MoonIOReal() {
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
  public void updateInputs(MoonIOInputs inputs) {
    inputs.connected = motor.getFirmwareVersion() != 0;
    inputs.appliedVolts = motor.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.currentAmps = motor.getOutputCurrent();
    inputs.positionRadians = encoder.getPosition();
    inputs.rpm = encoder.getVelocity();

    if (reverseLimitSwitch.isPressed()) {
      encoder.setPosition(0);
    }
  }
}
