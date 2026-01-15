package frc.robot.subsystems.moon;

import static frc.robot.subsystems.moon.MoonConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class MoonCtrlManual extends SubsystemBase implements MoonCtrl {
  private LoggedTunableNumber maxVoltage = new LoggedTunableNumber("Moon/MaxVoltage", MAX_VOLTAGE);
  private LoggedTunableNumber pGain = new LoggedTunableNumber("Moon/PGain", KP);
  private final MoonIO io;
  private final MoonIOInputsAutoLogged inputs;
  private final PIDController controller;
  private double targetRadians;

  public MoonCtrlManual(MoonIO io) {
    this.io = io;
    inputs = new MoonIOInputsAutoLogged();
    controller = new PIDController(pGain.get(), KI, KD);
    targetRadians = 0.0;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Moon", inputs);

    // PID change
    if (pGain.hasChanged(pGain.hashCode())) {
      controller.setP(KP);
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
    targetRadians = MathUtil.clamp(targetRadians + radians, 0, MAX_RADIANS);
  }
}
