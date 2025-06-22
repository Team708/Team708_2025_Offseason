import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberConstants.ClimberState;
import frc.robot.subsystems.climber.ClimberCtrl;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class ClimberTests {
  private Climber climber;

  @BeforeEach
  void setup() {
    climber = new Climber();
  }

  @Test
  void testInitialState() {
    ClimberCtrl climberCtrl = climber.getClimberCtrl();
    double totalTime = 1.0; // seconds
    double dt = 0.02; // 20ms loop
    int iterations = (int) (totalTime / dt);

    for (int i = 0; i < iterations; i++) {
      climber.periodic();
    }
    assertTrue(
        climberCtrl.getClimberState() == ClimberState.RETRACTED
            && climberCtrl.getDesiredClimberState() == ClimberState.RETRACTED);
  }

  @Test
  void testDeploy() {
    ClimberCtrl climberCtrl = climber.getClimberCtrl();
    double totalTime = 1.0; // seconds
    double dt = 0.02; // 20ms loop
    int iterations = (int) (totalTime / dt);

    for (int i = 0; i < iterations; i++) {
      climberCtrl.periodic();
    }
    totalTime = 5.0;
    iterations = (int) (totalTime / dt);
    climberCtrl.startClimb();
    for (int i = 0; i < iterations; i++) {
      climberCtrl.periodic();
    }
    assertTrue(climberCtrl.getClimberState() == ClimberState.EXTENDED);
  }
}
