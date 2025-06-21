import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeCtrlSystem;
import frc.robot.subsystems.intake.IntakeCtrlSystem.IntakeMode;
import frc.robot.subsystems.intake.IntakeIOSim;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class IntakeTests {
  private Intake intake;
  private IntakeCtrlSystem intakeCtrl;
  private IntakeIOSim intakeSim;

  @BeforeEach
  void setup() {
    intakeCtrl = new IntakeCtrlSystem();
    intake = new Intake(intakeCtrl);
  }

  @Test
  void testSetMode() {
    intakeCtrl.setMode(IntakeMode.CORAL_OUTAKE);
    assertTrue(intakeCtrl.getMode() == IntakeMode.CORAL_OUTAKE);
  }
}
