package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class ElevatorCtrlManual extends SubsystemBase implements ElevatorCtrl {
  private LoggedTunableNumber maxVoltage =
      new LoggedTunableNumber("Elevator/MaxVoltage", kMaxVoltage);
  private LoggedTunableNumber pGain = new LoggedTunableNumber("Elevator/PGain", kP);
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs;
  private final PIDController controller;
  private double targetInches;

  public ElevatorCtrlManual(ElevatorIO io) {
    this.io = io;
    inputs = new ElevatorIOInputsAutoLogged();
    controller = new PIDController(pGain.get(), kI, kD);
    targetInches = 0.0;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    // PID change
    if (pGain.hasChanged(pGain.hashCode())) {
      controller.setP(kP);
    }

    // Scale PID to voltage output
    double rawPID = controller.calculate(inputs.positionInches, targetInches);
    double scaledVoltage = MathUtil.clamp(rawPID, -maxVoltage.get(), maxVoltage.get());
    io.setVoltage(scaledVoltage);
  }

  @Override
  public void manualAdjustPosition(double inches) {
    targetInches = MathUtil.clamp(targetInches + inches, 0, kMaxHeightInches);
  }
}
