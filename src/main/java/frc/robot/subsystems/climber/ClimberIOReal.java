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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants;

public class ClimberIOReal implements ClimberIO {
  private final Servo servo;
  private boolean isServoUnlocked;
  private final SparkFlex motor;
  private final SparkLimitSwitch reverseLimitSwitch;
  private final RelativeEncoder encoder;
  private final DigitalInput cageLimit1;
  private final DigitalInput cageLimit2;
  private final DigitalInput beamBreak1;
  private final DigitalInput beamBreak2;

  // Backgrounded operations
  private final Notifier backgroundThread;
  private boolean isMotorConnected;

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
        .inverted(true)
        .smartCurrentLimit(kCurrentLimit)
        .voltageCompensation(12.0);
    motorConfig
        .encoder
        .positionConversionFactor(kEncoderPositionFactor)
        .velocityConversionFactor(kEncoderVelocityFactor)
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
    cageLimit1 = new DigitalInput(kCageLimitSwitch1);
    cageLimit2 = new DigitalInput(kCageLimitSwitch2);
    beamBreak1 = new DigitalInput(kBeamBreak1);
    beamBreak2 = new DigitalInput(kBeamBreak2);
    backgroundThread = new Notifier(this::updateBackground);
    backgroundThread.startPeriodic(Constants.backgroundThreadPeriod);
  }

  private void updateBackground() {
    isMotorConnected = motor.getFirmwareVersion() != 0;
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.servoUnlocked = isServoUnlocked;
    inputs.connected = isMotorConnected;
    inputs.appliedVolts = motor.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.currentAmps = motor.getOutputCurrent();
    inputs.positionRadians = encoder.getPosition();
    inputs.positionDegrees = Math.toDegrees(inputs.positionRadians);
    inputs.rpm = encoder.getVelocity();
    inputs.cageLimit1 = cageLimit1.get();
    inputs.cageLimit2 = cageLimit2.get();
    inputs.beamBreak1 = beamBreak1.get();
    inputs.beamBreak2 = beamBreak2.get();

    if (reverseLimitSwitch.isPressed()) {
      inputs.reverseLimitReached = true;
      encoder.setPosition(0);
    } else {
      inputs.reverseLimitReached = false;
    }
    if (inputs.positionRadians >= (kExtendedRadians - kDeadband)) {
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
