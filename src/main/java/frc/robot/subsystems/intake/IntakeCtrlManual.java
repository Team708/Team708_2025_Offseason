package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IntakeCtrlManual extends SubsystemBase implements IntakeCtrl {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs;

  public IntakeCtrlManual(IntakeIO io) {
    this.io = io;
    inputs = new IntakeIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }
}
