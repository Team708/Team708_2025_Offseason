package frc.robot.subsystems.moon;

import static frc.robot.subsystems.moon.MoonConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.moon.MoonConstants.MoonTarget;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class MoonCtrlSystem extends SubsystemBase implements MoonCtrl {
  private LoggedTunableNumber maxVoltage = new LoggedTunableNumber("Moon/MaxVoltage", kMaxVoltage);
  private LoggedTunableNumber pGain = new LoggedTunableNumber("Moon/PGain", kP);
  private LoggedTunableNumber zeroingVoltage =
      new LoggedTunableNumber("Elevator/ZeroingVoltage", kZeroingVoltage);
  private final MoonIO io;
  private final MoonIOInputsAutoLogged inputs;
  private final PIDController controller;
  private boolean isCoralMode;
  private double targetRadians;

  public MoonCtrlSystem(MoonIO io) {
    this.io = io;
    inputs = new MoonIOInputsAutoLogged();
    controller = new PIDController(pGain.get(), kI, kD);
    targetRadians = 0.0;
    isCoralMode = true;
    Logger.recordOutput("Moon/TargetPosition", "Unknown");
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Moon", inputs);
    Logger.recordOutput("Moon/IsAtTarget", atTargetPosition());

    // PID change
    if (pGain.hasChanged(pGain.hashCode())) {
      controller.setP(kP);
    }

    // Zeroing logic
    if (!inputs.reverseLimitReached
        && targetRadians == 0
        && inputs.appliedVolts < zeroingVoltage.get()) {
      io.setVoltage(zeroingVoltage.get());
      return;
    }

    // Scale PID to voltage output
    double rawPID = controller.calculate(inputs.positionRadians, targetRadians);
    double scaledVoltage = MathUtil.clamp(rawPID, -maxVoltage.get(), maxVoltage.get());
    io.setVoltage(scaledVoltage);
  }

  @Override
  public void setTargetPosition(MoonTarget target) {
    Logger.recordOutput("Moon/TargetPosition", target.name());
    targetRadians = target.radians;
  }

  @Override
  public boolean atTargetPosition() {
    return Math.abs(inputs.positionRadians - targetRadians) <= kDeadband;
  }

  @Override
  public boolean getIsCoralMode() {
    return isCoralMode;
  }

  @Override
  public void setIsCoralMode(boolean isCoralMode) {
    this.isCoralMode = isCoralMode;
  }
}
