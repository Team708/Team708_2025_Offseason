package frc.robot.subsystems.chute;

import static frc.robot.subsystems.chute.ChuteConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Chute extends SubsystemBase {

  private enum State {
    UNKNOWN,
    MOVING,
    RETRACTED,
    EXTENDED
  }

  @AutoLogOutput private State state;
  @AutoLogOutput private State desiredState;
  private final ChuteIO io;
  private final ChuteIOInputsAutoLogged inputs;
  private final PIDController controller;

  public Chute(ChuteIO io) {
    this.io = io;
    inputs = new ChuteIOInputsAutoLogged();
    controller = new PIDController(kP, kI, kD);
    state = State.UNKNOWN;
    desiredState = State.RETRACTED;
  }

  public void retract() {
    desiredState = State.RETRACTED;
  }

  public void extend() {
    desiredState = State.EXTENDED;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Chute", inputs);
    if (inputs.isFullyRetracted) {
      state = State.RETRACTED;
    } else if (inputs.isFullyExtended) {
      state = State.EXTENDED;
    }

    double desiredPosition;
    if (state != desiredState) {
      switch (desiredState) {
        case UNKNOWN:
          desiredPosition = kRetractedMeters;
          break;
        case RETRACTED:
          desiredPosition = kRetractedMeters;
          break;
        case EXTENDED:
          desiredPosition = kExtendedMeters;
          break;
        default:
          desiredPosition = kRetractedMeters;
          break;
      }
      state = State.MOVING;
      double outputVolts = controller.calculate(inputs.positionMeters, desiredPosition);
      io.setVoltage(MathUtil.clamp(outputVolts, -12.0, 12.0));
    } else {
      io.setVoltage(0.0);
    }
  }

  public boolean isExtended() {
    return inputs.isFullyExtended;
  }

  public boolean isRetracted() {
    return inputs.isFullyRetracted;
  }
}
