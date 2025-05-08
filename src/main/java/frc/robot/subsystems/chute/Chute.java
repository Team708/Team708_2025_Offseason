package frc.robot.subsystems.chute;

import static frc.robot.subsystems.chute.ChuteConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Chute extends SubsystemBase {

  private enum State {
    UNKNOWN,
    RETRACTED,
    EXTENDED
  }

  private State state;
  private State desiredState;
  private final ChuteIO io;
  private final ChuteIOInputsAutoLogged inputs;
  private final PIDController controller;
  private double zeroOffset;
  private double goalMeters;

  public Chute(ChuteIO io) {
    this.io = io;
    inputs = new ChuteIOInputsAutoLogged();
    controller = new PIDController(kP, kI, kD);
    controller.enableContinuousInput(-12.0, 12.0);
    zeroOffset = 0.0;
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
      double outputVolts = controller.calculate(inputs.positionMeters, desiredPosition);
      io.setVoltage(outputVolts);
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
