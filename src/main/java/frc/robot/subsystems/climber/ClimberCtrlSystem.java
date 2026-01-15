package frc.robot.subsystems.climber;

import static frc.robot.subsystems.chute.ChuteConstants.KI;
import static frc.robot.subsystems.chute.ChuteConstants.KP;
import static frc.robot.subsystems.climber.ClimberConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climber.ClimberConstants.ClimberState;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class ClimberCtrlSystem extends SubsystemBase implements ClimberCtrl {
  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs;
  private static final LoggedTunableNumber zeroingVolts =
      new LoggedTunableNumber("Climber/ZeroingVolts", ZEROING_VOLTAGE);
  private final PIDController controller;
  private ClimberState currentState;
  private ClimberState desiredState;

  public ClimberCtrlSystem(ClimberIO io) {
    this.io = io;
    inputs = new ClimberIOInputsAutoLogged();
    controller = new PIDController(KP, KI, KD);
    this.io.setServo(true);
    currentState = ClimberState.UNKNOWN;
    desiredState = ClimberState.RETRACTED;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
    Logger.recordOutput("Climber/CurrentClimberState", currentState.name());
    Logger.recordOutput("Climber/DesiredClimberState", desiredState.name());
    // ClimberState changes
    if (inputs.reverseLimitReached && currentState == ClimberState.UNKNOWN) {
      currentState = ClimberState.RETRACTED;
    }
    if (inputs.forwardLimitReached && currentState == ClimberState.RETRACTED) {
      currentState = ClimberState.EXTENDED;
      desiredState = ClimberState.ENGAGED;
    }
    if (!inputs.beamBreak1
        && !inputs.beamBreak2
        && !inputs.cageLimit1
        && !inputs.cageLimit2
        && desiredState == ClimberState.ENGAGED
        && currentState == ClimberState.EXTENDED) {
      currentState = ClimberState.ENGAGED;
      desiredState = ClimberState.CLIMBED;
    }
    if (desiredState == ClimberState.CLIMBED
        && currentState == ClimberState.ENGAGED
        && inputs.reverseLimitReached) {
      currentState = ClimberState.CLIMBED;
    }

    // What to do in each state
    switch (currentState) {
      case UNKNOWN:
        io.setServo(true);
        if (desiredState == ClimberState.RETRACTED) {
          io.setVoltage(zeroingVolts.get());
        }
        return;
      case RETRACTED:
        if (desiredState == ClimberState.EXTENDED) {
          pidSetPoint(EXTENDED_RADIANS);
        } else {
          pidSetPoint(-1.0);
        }
        return;
      case EXTENDED:
        pidSetPoint(EXTENDED_RADIANS);
        return;
      case ENGAGED:
        if (desiredState == ClimberState.ENGAGED) {
          pidSetPoint(EXTENDED_RADIANS);
        } else if (desiredState == ClimberState.CLIMBED) {
          pidSetPoint(-1.0);
        }
        return;
      case CLIMBED:
        io.setVoltage(0.0);
        io.setServo(false);
        return;
    }
  }

  private void pidSetPoint(double targetSetpoint) {
    double outputVolts = controller.calculate(inputs.positionRadians, targetSetpoint);
    io.setVoltage(MathUtil.clamp(outputVolts, -MAX_VOLTAGE, MAX_VOLTAGE));
  }

  @Override
  public void startClimb() {
    if (currentState == ClimberState.RETRACTED && desiredState == ClimberState.RETRACTED) {
      io.setServo(true);
      desiredState = ClimberState.EXTENDED;
    }
  }

  @Override
  public boolean readyToClimb() {
    if (currentState == ClimberState.RETRACTED && desiredState == ClimberState.RETRACTED) {
      return true;
    } else {
      return false;
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

  @Override
  public ClimberState getClimberState() {
    return currentState;
  }

  @Override
  public ClimberState getDesiredClimberState() {
    return desiredState;
  }
}
