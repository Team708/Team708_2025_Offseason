package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class IntakeCtrlSystem extends SubsystemBase {
  public enum IntakeMode {
    STOP,
    CORAL_INTAKE,
    CORAL_OUTAKE,
    ALGAE_INTAKE,
    ALGAE_OUTAKE
  }

  private LoggedTunableNumber pGain = new LoggedTunableNumber("Intake/PGain", kP);
  private LoggedTunableNumber holdingVoltage =
      new LoggedTunableNumber("Intake/HoldingVoltage", kHoldingVoltage);
  private LoggedTunableNumber coralIntakeVoltage =
      new LoggedTunableNumber("Intake/CoralIntakeVoltage", kCoralIntakeVoltage);
  private LoggedTunableNumber coralOutakeVoltage =
      new LoggedTunableNumber("Intake/CoralOutakeVoltage", kCoralOutakeVoltage);
  private LoggedTunableNumber algaeIntakeVoltage =
      new LoggedTunableNumber("Intake/AlgaeIntakeVoltage", kAlgaeIntakeVoltage);
  private LoggedTunableNumber algaeOutakeVoltage =
      new LoggedTunableNumber("Intake/AlgaeOutakeVoltage", kAlgaeOutakeVoltage);
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs;
  public PIDController controller;
  private IntakeMode mode;
  private boolean holdingEnabled;
  private double targetHoldPosRad;

  public IntakeCtrlSystem(IntakeIO io) {
    this.io = io;
    inputs = new IntakeIOInputsAutoLogged();
    mode = IntakeMode.STOP;
    controller = new PIDController(kP, kI, kD);
    holdingEnabled = false;
    targetHoldPosRad = 0.0;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    Logger.recordOutput("Intake/TargetHold", targetHoldPosRad);

    // PID change
    if (pGain.hasChanged(pGain.hashCode())) {
      controller.setP(kP);
    }

    if (!holdingEnabled) {
      switch (mode) {
        case STOP:
          io.setVoltage(0.0);
          break;
        case CORAL_INTAKE:
          io.setVoltage(coralIntakeVoltage.get());
          break;
        case CORAL_OUTAKE:
          io.setVoltage(coralOutakeVoltage.get());
          break;
        case ALGAE_INTAKE:
          io.setVoltage(algaeIntakeVoltage.get());
          break;
        case ALGAE_OUTAKE:
          io.setVoltage(algaeOutakeVoltage.get());
          break;
        default:
          io.setVoltage(0.0);
          break;
      }
    } else {
      // Scale PID to voltage output
      double rawPID = controller.calculate(inputs.positionRad, targetHoldPosRad);
      double scaledVoltage = MathUtil.clamp(rawPID, -holdingVoltage.get(), holdingVoltage.get());
      io.setVoltage(scaledVoltage);
    }
  }

  public void holdCurrentPosition() {
    targetHoldPosRad = inputs.positionRad;
    System.out.println("HOLD CALL");
    holdingEnabled = true;
  }

  public void disableHold() {
    holdingEnabled = false;
  }

  public void setMode(IntakeMode mode) {
    this.mode = mode;
  }

  public boolean hasCoral() {
    return inputs.beamTriggered;
  }
}
