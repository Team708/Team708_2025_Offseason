package frc.robot.subsystems.moon;

import static frc.robot.subsystems.moon.MoonConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class MoonCtrlManual extends MoonCtrlBase implements MoonCtrl {
  private LoggedTunableNumber maxVoltage = new LoggedTunableNumber("Moon/MaxVoltage", kMaxVoltage);
  private LoggedTunableNumber pGain = new LoggedTunableNumber("Moon/PGain", kP);

  private final PIDController controller = new PIDController(pGain.get(), kI, kD);
  ;
  private double targetRadians;

  @Override
  protected void init() {
    targetRadians = 0.0;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Moon", inputs);

    // PID change
    if (pGain.hasChanged(pGain.hashCode())) {
      controller.setP(kP);
    }

    // Scale PID to voltage output
    double rawPID = controller.calculate(inputs.positionRadians * 12, targetRadians * 12);
    double scaledVoltage = MathUtil.clamp(rawPID, -maxVoltage.get(), maxVoltage.get());
    io.setVoltage(scaledVoltage);
  }

  @Override
  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  @Override
  public void manualAdjust(double radians) {
    targetRadians = MathUtil.clamp(targetRadians + radians, 0, kMaxRadians);
  }
}
