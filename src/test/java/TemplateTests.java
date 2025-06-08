import frc.robot.subsystems.template.Template;
import frc.robot.subsystems.template.TemplateCtrl;
import frc.robot.subsystems.template.TemplateCtrlSystem;
import frc.robot.subsystems.template.TemplateIOSim;
import org.junit.jupiter.api.BeforeEach;

public class TemplateTests {
  private Template template;
  private TemplateCtrl templateCtrl;
  private TemplateIOSim templateSim;

  @BeforeEach
  void setup() {
    templateSim = new TemplateIOSim();
    templateCtrl = new TemplateCtrlSystem(templateSim);
    template = new Template(templateCtrl);
  }
}
