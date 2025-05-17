package frc.robot.subsystems.template;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class TemplateCtrlManual extends SubsystemBase implements TemplateCtrl {
  private final TemplateIO io;
  private final TemplateIOInputsAutoLogged inputs;

  public TemplateCtrlManual(TemplateIO io) {
    this.io = io;
    inputs = new TemplateIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Template", inputs);
  }
}
