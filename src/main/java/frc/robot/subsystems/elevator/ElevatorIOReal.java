package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;

public class ElevatorIOReal implements ElevatorIO {
  private final SparkFlex motorLeader;
  private final SparkFlex motorFollower;
  private final RelativeEncoder encoder;
  private final SparkLimitSwitch reverseLimitSwitch;
  private final SparkClosedLoopController controller;

  // Backgrounded operations
  private final Notifier backgroundThread;
  private boolean isLeaderConnected;
  private boolean isFollowerConnected;

  public ElevatorIOReal() {
    motorLeader = new SparkFlex(CAN_ID_MOTOR_1, MotorType.kBrushless);
    motorFollower = new SparkFlex(CAN_ID_MOTOR_2, MotorType.kBrushless);
    encoder = motorLeader.getEncoder();

    // Leader
    var leaderConfig = new SparkFlexConfig();
    leaderConfig
        .idleMode(IdleMode.kBrake)
        .inverted(true)
        .smartCurrentLimit(CURRENT_LIMIT)
        .voltageCompensation(12.0);
    leaderConfig
        .encoder
        .positionConversionFactor(POSITION_FACTOR)
        .velocityConversionFactor(VELOCITY_FACTOR)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    leaderConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(KP)
        .d(KD)
        .i(KI)
        .outputRange(MIN_CLOSED_LOOP_OUTPUT, MAX_CLOSED_LOOP_OUTPUT);
    leaderConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (1000.0 / 100.0))
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        motorLeader,
        5,
        () ->
            motorLeader.configure(
                leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(motorLeader, 5, () -> encoder.setPosition(0.0));

    // Follower
    var followerConfig = new SparkFlexConfig();
    followerConfig.inverted(false);
    followerConfig.follow(CAN_ID_MOTOR_1, true);
    tryUntilOk(
        motorFollower,
        5,
        () ->
            motorFollower.configure(
                followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    reverseLimitSwitch = motorLeader.getReverseLimitSwitch();
    controller = motorLeader.getClosedLoopController();
    backgroundThread = new Notifier(this::updateBackground);
    backgroundThread.startPeriodic(Constants.BACKGROUND_THREAD_PERIOD);
  }

  private void updateBackground() {
    isLeaderConnected = motorLeader.getFirmwareVersion() != 0;
    isFollowerConnected = motorFollower.getFirmwareVersion() != 0;
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    // inputs.motor1Connected = motorLeader.getFirmwareVersion() != 0;
    // inputs.motor2Connected = motorFollower.getFirmwareVersion() != 0;
    inputs.motor1Connected = isLeaderConnected;
    inputs.motor2Connected = isFollowerConnected;
    inputs.reverseLimitTriggered = reverseLimitSwitch.isPressed();
    inputs.appliedVolts = motorLeader.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.currentAmps = motorLeader.getOutputCurrent();
    inputs.positionInches = encoder.getPosition();
    inputs.velocityInchesPerSecond = encoder.getVelocity();
    controller.setReference(inputs.targetInches, ControlType.kPosition);
  }

  @Override
  public void setVoltage(double volts) {
    motorLeader.setVoltage(volts);
  }
}
