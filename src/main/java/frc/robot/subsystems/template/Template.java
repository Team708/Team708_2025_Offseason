package frc.robot.subsystems.template;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Template extends SubsystemBase {
  private TemplateCtrl templateCtrl;

  public Template(TemplateCtrl templateCtrl) {
    this.templateCtrl = templateCtrl;
  }

  @Override
  public void periodic() {
    templateCtrl.periodic();
  }

  public TemplateCtrl getTemplateCtrl() {
    return this.templateCtrl;
  }
}
