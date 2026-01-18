package frc.robot.subsystems.chute;

import static frc.robot.subsystems.chute.ChuteConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;

public class ChuteIOReal implements ChuteIO {
  private final SparkFlex motor;
  private final RelativeEncoder encoder;
  private final SparkLimitSwitch reverseLimitSwitch;
  private final SparkLimitSwitch forwardLimitSwitch;

  // Backgrounded operations
  private final Notifier backgroundThread;
  private boolean isMotorConnected;

  public ChuteIOReal() {
    motor = new SparkFlex(ChuteConstants.CAN_ID, MotorType.kBrushless);
    encoder = motor.getEncoder();

    // Configure drive motor
    var motorConfig = new SparkFlexConfig();
    motorConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(ChuteConstants.CURRENT_LIMIT)
        .voltageCompensation(12.0);
    motorConfig
        .encoder
        .positionConversionFactor(ChuteConstants.ENCODER_POSITION_FACTOR)
        .velocityConversionFactor(ChuteConstants.ENCODER_VELOCITY_FACTOR)
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
    forwardLimitSwitch = motor.getForwardLimitSwitch();
    backgroundThread = new Notifier(this::updateBackground);
    backgroundThread.startPeriodic(Constants.BACKGROUND_THREAD_PERIOD);
  }

  private void updateBackground() {
    isMotorConnected = motor.getFirmwareVersion() != 0;
  }

  @Override
  public void updateInputs(ChuteIOInputs inputs) {
    // inputs.connected = motor.getFirmwareVersion() != 0;
    inputs.connected = isMotorConnected;
    inputs.isFullyRetracted = reverseLimitSwitch.isPressed();
    inputs.isFullyExtended = forwardLimitSwitch.isPressed();
    inputs.appliedVolts = motor.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.currentAmps = motor.getOutputCurrent();
    inputs.positionInches = encoder.getPosition();
    inputs.velocityInchesPerSecond = encoder.getVelocity();
    inputs.rpm = (encoder.getVelocity() / SCREW_INCHES_PER_REV) * 60;

    if (inputs.isFullyRetracted) {
      encoder.setPosition(0.0);
    }
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }
}
