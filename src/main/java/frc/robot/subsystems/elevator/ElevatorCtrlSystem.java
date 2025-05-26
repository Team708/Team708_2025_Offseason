package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class ElevatorCtrlSystem extends SubsystemBase implements ElevatorCtrl {
  private LoggedTunableNumber maxVoltage =
      new LoggedTunableNumber("Elevator/MaxVoltage", kMaxVoltage);
  private LoggedTunableNumber pGain = new LoggedTunableNumber("Elevator/PGain", kP);
  private LoggedTunableNumber zeroingVoltage =
      new LoggedTunableNumber("Elevator/ZeroingVoltage", kZeroingVoltage);
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs;
  private final PIDController controller;
  private double targetMeters;

  public ElevatorCtrlSystem(ElevatorIO io) {
    this.io = io;
    inputs = new ElevatorIOInputsAutoLogged();
    controller = new PIDController(pGain.get(), kI, kD);
    targetMeters = 0.0;
    Logger.recordOutput("Elevator/TargetLevel", "Unknown");
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    Logger.recordOutput("Elevator/IsAtTarget", atTargetPosition());

    // PID change
    if (pGain.hasChanged(pGain.hashCode())) {
      controller.setP(kP);
    }

    // Zeroing logic
    if (!inputs.reverseLimitTriggered
        && targetMeters == 0
        && inputs.appliedVolts < zeroingVoltage.get()) {
      io.setVoltage(zeroingVoltage.get());
      return;
    }

    // Scale PID to voltage output
    double rawPID = controller.calculate(inputs.positionMeters, targetMeters);
    double scaledVoltage = MathUtil.clamp(rawPID, -maxVoltage.get(), maxVoltage.get());
    io.setVoltage(scaledVoltage);
  }

  @Override
  public void setTargetPosition(ElevatorTarget target) {
    Logger.recordOutput("Elevator/TargetLevel", target.name());
    targetMeters = target.heightMeters;
  }

  @Override
  public boolean atTargetPosition() {
    return Math.abs(inputs.positionMeters - targetMeters) <= kDeadband;
  }
}
