package frc.robot.subsystems.chute;

import static frc.robot.subsystems.chute.ChuteConstants.*;
import static frc.robot.util.SparkUtil.*;

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

public class ChuteIOReal implements ChuteIO {
  private final SparkFlex motor;
  private final RelativeEncoder encoder;
  private final SparkLimitSwitch reverseLimitSwitch;
  private final SparkLimitSwitch forwardLimitSwitch;

  public ChuteIOReal() {
    motor = new SparkFlex(ChuteConstants.kCanID, MotorType.kBrushless);
    encoder = motor.getEncoder();

    // Configure drive motor
    var motorConfig = new SparkFlexConfig();
    motorConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(ChuteConstants.kCurrentLimit)
        .voltageCompensation(12.0);
    motorConfig
        .encoder
        .positionConversionFactor(ChuteConstants.kEncoderPositionFactor)
        .velocityConversionFactor(ChuteConstants.kEncoderVelocityFactor)
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
  }

  @Override
  public void updateInputs(ChuteIOInputs inputs) {
    inputs.connected = motor.getFirmwareVersion() != 0;
    inputs.isFullyRetracted = reverseLimitSwitch.isPressed();
    inputs.isFullyExtended = forwardLimitSwitch.isPressed();
    inputs.appliedVolts = motor.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.currentAmps = motor.getOutputCurrent();
    inputs.positionMeters = encoder.getPosition();
    inputs.velocityMetersPerSecond = encoder.getVelocity();
    inputs.rpm = (encoder.getVelocity() / kScrewTravelPerRev) * 60;

    if (inputs.isFullyRetracted) {
      encoder.setPosition(0.0);
    }
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }
}
