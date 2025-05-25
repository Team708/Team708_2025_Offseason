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
  private double targetMeters;

  public ElevatorCtrlManual(ElevatorIO io) {
    this.io = io;
    inputs = new ElevatorIOInputsAutoLogged();
    controller = new PIDController(pGain.get(), kI, kD);
    targetMeters = 0.0;
  }

  @Override
  public void periodic() {
    // PID chang
    if (pGain.hasChanged(pGain.hashCode())) {
      controller.setP(kP);
    }

    // Scale PID to voltage output
    double rawPID = controller.calculate(inputs.positionMeters, targetMeters);
    double scaledVoltage = MathUtil.clamp(rawPID, -maxVoltage.get(), maxVoltage.get());

    Logger.recordOutput("Elevator/rawPID", rawPID);
    Logger.recordOutput("Elevator/finalVoltage", scaledVoltage);
    Logger.recordOutput("Elevator/target", targetMeters);

    io.setVoltage(scaledVoltage);
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
  }

  @Override
  public void changeElevatorPosition(double meters) {
    targetMeters += meters;
    if (targetMeters < 0) {
      targetMeters = 0;
    } else if (targetMeters > kMaxHeightMeters) {
      targetMeters = kMaxHeightMeters;
    }
  }
}
