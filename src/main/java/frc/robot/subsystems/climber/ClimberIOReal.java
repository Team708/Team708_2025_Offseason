package frc.robot.subsystems.climber;

import static frc.robot.subsystems.climber.ClimberConstants.*;
import static frc.robot.subsystems.intake.IntakeConstants.CAN_ID;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.FeedbackSensor;
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
    servo = new Servo(SERVO_CHANNEL);
    isServoUnlocked = true;
    setServo(true);
    motor = new SparkFlex(CAN_ID, MotorType.kBrushless);
    encoder = motor.getEncoder();

    // Configure drive motor
    var motorConfig = new SparkFlexConfig();
    motorConfig
        .idleMode(IdleMode.kBrake)
        .inverted(true)
        .smartCurrentLimit(CURRENT_LIMIT)
        .voltageCompensation(12.0);
    motorConfig
        .encoder
        .positionConversionFactor(ENCODER_POSITION_FACTOR)
        .velocityConversionFactor(ENCODER_VELOCITY_FACTOR)
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
    cageLimit1 = new DigitalInput(CAGE_LIMIT_SWITCH_1_CHANNEL);
    cageLimit2 = new DigitalInput(CAGE_LIMIT_SWITCH_2_CHANNEL);
    beamBreak1 = new DigitalInput(BEAM_BREAK_1_CHANNEL);
    beamBreak2 = new DigitalInput(BEAM_BRAKE_2_CHANNEL);
    backgroundThread = new Notifier(this::updateBackground);
    backgroundThread.startPeriodic(Constants.BACKGROUND_THREAD_PERIOD);
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
    if (inputs.positionRadians >= (EXTENDED_RADIANS - DEADBAND)) {
      inputs.forwardLimitReached = true;
    } else {
      inputs.forwardLimitReached = false;
    }
  }

  @Override
  public void setServo(boolean isUnlocked) {
    if (isUnlocked) {
      servo.setPosition(SERVO_RELEASE_POSITION);
    } else {
      servo.setPosition(SERVO_BRAKE_POSITION);
    }
    isServoUnlocked = isUnlocked;
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }
}
