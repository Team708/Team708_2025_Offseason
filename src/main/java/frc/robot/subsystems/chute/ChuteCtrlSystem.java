package frc.robot.subsystems.chute;

import static frc.robot.subsystems.chute.ChuteConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ChuteCtrlSystem extends ChuteCtrlBase implements ChuteCtrl {

  private enum State {
    UNKNOWN,
    MOVING,
    RETRACTED,
    EXTENDED
  }

  @AutoLogOutput private State state;
  @AutoLogOutput private State desiredState;
  private final PIDController controller = new PIDController(pGain.get(), kI, kD);
  ;
  private static final LoggedTunableNumber maxVolts =
      new LoggedTunableNumber("Chute/Volts", kMaxVoltage);
  private static final LoggedTunableNumber pGain = new LoggedTunableNumber("Chute/P", kP);
  private static final LoggedTunableNumber zeroingVolts =
      new LoggedTunableNumber("Chute/ZeroingVolts", kZeroingVoltage);

  public void retract() {
    desiredState = State.RETRACTED;
  }

  public void extend() {
    desiredState = State.EXTENDED;
  }

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
          desiredPosition = kRetractedInches;
          io.setVoltage(zeroingVolts.get());
          return;
        case RETRACTED:
          desiredPosition = kRetractedInches;
          break;
        case EXTENDED:
          desiredPosition = kExtendedInches;
          break;
        default:
          desiredPosition = kRetractedInches;
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

  @Override
  protected void init() {
    state = State.UNKNOWN;
    desiredState = State.RETRACTED;
  }
}
