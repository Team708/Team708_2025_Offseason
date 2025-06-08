import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeCtrlSystem;
import frc.robot.subsystems.intake.IntakeIOSim;
import org.junit.jupiter.api.BeforeEach;

public class IntakeTests {
  private Intake intake;
  private IntakeCtrlSystem intakeCtrl;
  private IntakeIOSim intakeSim;

  @BeforeEach
  void setup() {
    intakeSim = new IntakeIOSim();
    intakeCtrl = new IntakeCtrlSystem(intakeSim);
    intake = new Intake(intakeCtrl);
  }
}
