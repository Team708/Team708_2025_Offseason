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
  private LoggedTunableNumber pGain =
      new LoggedTunableNumber("Elevator/PGain", kP);
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs;
  private PIDController controller;
  private double targetMeters;

  public ElevatorCtrlManual(ElevatorIO io) {
    this.io = io;
    inputs = new ElevatorIOInputsAutoLogged();
    controller = new PIDController(pGain.get(), kI, kD);
    targetMeters = 0.0;
  }

  @Override
  public void periodic() {
    // PID change
    if(pGain.hasChanged(pGain.hashCode())) {
      controller = new PIDController(pGain.get(), kI, kD);
    }

    // Scale PID to voltage output
    double maxOutput = pGain.get() * (kMaxHeightMeters - kMinHeightMeters);
    double rawPID = controller.calculate(inputs.positionMeters, targetMeters);
    double scaledOutput = MathUtil.clamp(rawPID / maxOutput, -1.0, 1.0) * maxVoltage.get();

    io.setVoltage(scaledOutput);
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
  }

  @Override
  public void changeElevatorPosition(double meters) {
    targetMeters += meters;
  }
}
