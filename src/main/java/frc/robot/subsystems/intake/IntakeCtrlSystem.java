package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class IntakeCtrlSystem extends IntakeCtrlBase {
  public enum IntakeMode {
    STOP,
    CORAL_INTAKE,
    CORAL_OUTAKE,
    ALGAE_INTAKE,
    ALGAE_OUTAKE
  }

  private LoggedTunableNumber holdingVoltage =
      new LoggedTunableNumber("Intake/HoldingVoltage", kHoldingVoltage);
  public PIDController controller = new PIDController(kP, kI, kD);
  private IntakeMode mode;
  private boolean holdingEnabled;
  private double targetHoldPosRad;
  private double rawPID;
  private double scaledVoltage;

  protected void init() {
    mode = IntakeMode.STOP;
    holdingEnabled = false;
    targetHoldPosRad = 0.0;
    rawPID = 0.0;
    scaledVoltage = 0.0;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    Logger.recordOutput("Intake/hasCoral", hasCoral());

    if (!holdingEnabled) {
      switch (mode) {
        case STOP:
          io.setVoltage(0.0);
          break;
        case CORAL_INTAKE:
          io.setVoltage(kCoralIntakeVoltage);
          break;
        case CORAL_OUTAKE:
          io.setVoltage(kCoralOutakeVoltage);
          break;
        case ALGAE_INTAKE:
          io.setVoltage(kAlgaeIntakeVoltage);
          break;
        case ALGAE_OUTAKE:
          io.setVoltage(kAlgaeOutakeVoltage);
          break;
        default:
          io.setVoltage(0.0);
          break;
      }
    } else {
      // Scale PID to voltage output
      rawPID = controller.calculate(inputs.positionRad, targetHoldPosRad);
      scaledVoltage = MathUtil.clamp(rawPID, -holdingVoltage.get(), holdingVoltage.get());
      io.setVoltage(scaledVoltage);
    }
  }

  public void holdCurrentPosition() {
    targetHoldPosRad = inputs.positionRad - 0.5;
    holdingEnabled = true;
  }

  public void disableHold() {
    holdingEnabled = false;
  }

  public void setMode(IntakeMode mode) {
    this.mode = mode;
  }

  public IntakeMode getMode() {
    return this.mode;
  }

  public boolean hasCoral() {
    return inputs.beamTriggered;
  }

  public boolean hasAlgae() {
    return inputs.reverseLimitReached;
  }
}
