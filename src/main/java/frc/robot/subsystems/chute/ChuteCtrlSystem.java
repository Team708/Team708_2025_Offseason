package frc.robot.subsystems.chute;

import static frc.robot.subsystems.chute.ChuteConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ChuteCtrlSystem extends SubsystemBase implements ChuteCtrl {

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
      new LoggedTunableNumber("Chute/Volts", MAX_VOLTAGE);
  private static final LoggedTunableNumber pGain = new LoggedTunableNumber("Chute/P", KP);
  private static final LoggedTunableNumber zeroingVolts =
      new LoggedTunableNumber("Chute/ZeroingVolts", ZEROING_VOLTAGE);

  public ChuteCtrlSystem(ChuteIO io) {
    this.io = io;
    inputs = new ChuteIOInputsAutoLogged();
    controller = new PIDController(pGain.get(), KI, KD);
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
    } else if (inputs.isFullyExtended && state != State.UNKNOWN) {
      state = State.EXTENDED;
    }

    double desiredPosition;
    if (state != desiredState) {
      switch (desiredState) {
        case UNKNOWN:
          desiredPosition = RETRACTED_INCHES;
          io.setVoltage(zeroingVolts.get());
          return;
        case RETRACTED:
          desiredPosition = RETRACTED_INCHES;
          break;
        case EXTENDED:
          desiredPosition = EXTENDED_INCHES;
          break;
        default:
          desiredPosition = RETRACTED_INCHES;
          break;
      }
      state = State.MOVING;

      // Update controller P if needed
      if (pGain.hasChanged(pGain.hashCode())) {
        controller.setP(pGain.get());
      }
      double outputVolts = controller.calculate(inputs.positionInches, desiredPosition);
      io.setVoltage(MathUtil.clamp(outputVolts, -maxVolts.get(), maxVolts.get()));
    } else {
      io.setVoltage(0.0);
    }
  }

  @Override
  public boolean isExtended() {
    return inputs.isFullyExtended;
  }

  @Override
  public boolean isRetracted() {
    return inputs.isFullyRetracted;
  }

  public double getPosition() {
    return inputs.positionInches;
  }
}
