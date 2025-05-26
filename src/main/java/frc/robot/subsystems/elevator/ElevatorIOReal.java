package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;
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

public class ElevatorIOReal implements ElevatorIO {
  private final SparkFlex motorLeader;
  private final SparkFlex motorFollower;
  private final RelativeEncoder encoder;
  private final SparkLimitSwitch reverseLimitSwitch;

  public ElevatorIOReal() {
    motorLeader = new SparkFlex(kCanIDMotor1, MotorType.kBrushless);
    motorFollower = new SparkFlex(kCanIDMotor2, MotorType.kBrushless);
    encoder = motorLeader.getEncoder();

    // Leader
    var leaderConfig = new SparkFlexConfig();
    leaderConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(kCurrentLimit)
        .voltageCompensation(12.0);
    leaderConfig
        .encoder
        .positionConversionFactor(kPositionFactor)
        .velocityConversionFactor(kVelocityFactor)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    leaderConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
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
    followerConfig.follow(kCanIDMotor1);
    tryUntilOk(
        motorFollower,
        5,
        () ->
            motorFollower.configure(
                followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    reverseLimitSwitch = motorLeader.getReverseLimitSwitch();
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.motor1Connected = motorLeader.getFirmwareVersion() != 0;
    inputs.motor2Connected = motorFollower.getFirmwareVersion() != 0;
    inputs.reverseLimitTriggered = reverseLimitSwitch.isPressed();
    inputs.appliedVolts = motorLeader.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.currentAmps = motorLeader.getOutputCurrent();
    inputs.positionMeters = encoder.getPosition();
    inputs.velocityMetersPerSecond = encoder.getVelocity();
  }

  @Override
  public void setVoltage(double volts) {
    motorLeader.setVoltage(volts);
  }
}
