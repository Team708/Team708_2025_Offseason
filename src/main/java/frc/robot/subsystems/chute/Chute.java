package frc.robot.subsystems.chute;

import static frc.robot.subsystems.chute.ChuteConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
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
  @AutoLogOutput private boolean manualOverride;
  @AutoLogOutput private double manualVoltage;
  private static final LoggedTunableNumber maxVolts =
      new LoggedTunableNumber("Chute/Volts", kMaxVoltage);
  private static final LoggedTunableNumber pGain = new LoggedTunableNumber("Chute/P", kP);
  private static final LoggedTunableNumber zeroingVolts =
      new LoggedTunableNumber("Chute/ZeroingVolts", kZeroingVolts);

  public Chute(ChuteIO io) {
    this.io = io;
    inputs = new ChuteIOInputsAutoLogged();
    controller = new PIDController(pGain.get(), kI, kD);
    state = State.UNKNOWN;
    desiredState = State.RETRACTED;
    manualOverride = false;
    manualVoltage = 0.0;
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

    if (manualOverride) {
      io.setVoltage(manualVoltage);
      return;
    }

    if (inputs.isFullyRetracted) {
      state = State.RETRACTED;
    } else if (inputs.isFullyExtended && state != State.UNKNOWN) {
      state = State.EXTENDED;
    }

    double desiredPosition;
    if (state != desiredState) {
      switch (desiredState) {
        case UNKNOWN:
          desiredPosition = kRetractedMeters;
          io.setVoltage(zeroingVolts.get());
          return;
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

      // Update controller P if needed
      if (pGain.hasChanged(pGain.hashCode())) {
        controller.setP(pGain.get());
      }
      double outputVolts = controller.calculate(inputs.positionMeters, desiredPosition);
      io.setVoltage(MathUtil.clamp(outputVolts, -maxVolts.get(), maxVolts.get()));
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

  public void setManualOverride(boolean manualOverride) {
    this.manualOverride = manualOverride;
  }

  public void setManualVoltage(double manualVoltage) {
    this.manualVoltage = manualVoltage;
  }
}
