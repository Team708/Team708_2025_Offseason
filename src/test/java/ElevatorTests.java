import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorTarget;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class ElevatorTests {
  private Elevator elevator;

  @BeforeEach
  void setup() {
    elevator = new Elevator();
  }

  @Test
  void testMoveToPosition() {
    elevator.getElevatorCtrl().setTargetPosition(ElevatorTarget.CORAL_L4);
    double totalTime = 10.0; // seconds
    double dt = 0.02; // 20ms loop
    int iterations = (int) (totalTime / dt);

    for (int i = 0; i < iterations; i++) {
      elevator.periodic();
    }

    assertTrue(elevator.getElevatorCtrl().atTargetPosition());
  }
}
