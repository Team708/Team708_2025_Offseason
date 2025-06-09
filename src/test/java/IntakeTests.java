import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.subsystems.intake.IntakeCtrlSystem;
import frc.robot.subsystems.intake.IntakeCtrlSystem.IntakeMode;
import frc.robot.subsystems.intake.IntakeIOSim;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class IntakeTests {
  private IntakeCtrlSystem intakeCtrl;
  private IntakeIOSim intakeSim;

  @BeforeEach
  void setup() {
    intakeSim = new IntakeIOSim();
    intakeCtrl = new IntakeCtrlSystem(intakeSim);
  }

  @Test
  void testSetMode() {
    intakeCtrl.setMode(IntakeMode.CORAL_OUTAKE);
    assertTrue(intakeCtrl.getMode() == IntakeMode.CORAL_OUTAKE);
  }
}
