package frc.robot.subsystems.climber;

import static frc.robot.subsystems.chute.ChuteConstants.kI;
import static frc.robot.subsystems.chute.ChuteConstants.kP;
import static frc.robot.subsystems.climber.ClimberConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class ClimberCtrlSystem extends SubsystemBase implements ClimberCtrl {
  private enum State {
    UNKNOWN,
    RETRACTED,
    EXTENDED,
    ENGAGED,
    CLIMBED
  }

  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs;
  private static final LoggedTunableNumber zeroingVolts =
      new LoggedTunableNumber("Climber/ZeroingVolts", kZeroingVoltage);
  private final PIDController controller;
  private State currentState;
  private State desiredState;

  public ClimberCtrlSystem(ClimberIO io) {
    this.io = io;
    inputs = new ClimberIOInputsAutoLogged();
    controller = new PIDController(kP, kI, kD);
    this.io.setServo(true);
    currentState = State.UNKNOWN;
    desiredState = State.RETRACTED;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
    Logger.recordOutput("Climber/CurrentState", currentState.name());
    Logger.recordOutput("Climber/DesiredState", desiredState.name());
    // State changes
    if (inputs.reverseLimitReached && currentState == State.UNKNOWN) {
      currentState = State.RETRACTED;
    }
    if (inputs.forwardLimitReached && currentState == State.RETRACTED) {
      currentState = State.EXTENDED;
      desiredState = State.ENGAGED;
    }
    if (!inputs.beamBreak1
        && !inputs.beamBreak2
        && !inputs.cageLimit1
        && !inputs.cageLimit2
        && desiredState == State.ENGAGED
        && currentState == State.EXTENDED) {
      currentState = State.ENGAGED;
      desiredState = State.CLIMBED;
    }
    if (desiredState == State.CLIMBED
        && currentState == State.ENGAGED
        && inputs.reverseLimitReached) {
      currentState = State.CLIMBED;
    }

    double outputVolts;
    // What to do in each state
    switch (currentState) {
      case UNKNOWN:
        io.setServo(true);
        if (desiredState == State.RETRACTED) {
          io.setVoltage(zeroingVolts.get());
        }
        return;
      case RETRACTED:
        if (desiredState == State.EXTENDED) {
          outputVolts = controller.calculate(inputs.positionRadians, kExtendedRadians);
          io.setVoltage(MathUtil.clamp(outputVolts, -kMaxVoltage, kMaxVoltage));
        } else {
          outputVolts = controller.calculate(inputs.positionRadians, -1.0);
          io.setVoltage(MathUtil.clamp(outputVolts, -kMaxVoltage, kMaxVoltage));
        }
        return;
      case EXTENDED:
        outputVolts = controller.calculate(inputs.positionRadians, kExtendedRadians);
        io.setVoltage(MathUtil.clamp(outputVolts, -kMaxVoltage, kMaxVoltage));
        return;
      case ENGAGED:
        if (desiredState == State.ENGAGED) {
          io.setVoltage(0.0);
        } else if (desiredState == State.CLIMBED) {
          outputVolts = controller.calculate(inputs.positionRadians, -1.0);
          io.setVoltage(MathUtil.clamp(outputVolts, -kMaxVoltage, kMaxVoltage));
        }
        return;
      case CLIMBED:
        io.setVoltage(0.0);
        io.setServo(false);
        return;
    }
  }

  public void startClimb() {
    if (currentState == State.RETRACTED && desiredState == State.RETRACTED) {
      io.setServo(true);
      desiredState = State.EXTENDED;
    }
  }

  @Override
  public boolean isAtDesiredState() {
    if (currentState == desiredState) {
      return true;
    } else {
      return false;
    }
  }
}
