import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorTarget;
import frc.robot.subsystems.elevator.ElevatorCtrlSystem;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class ElevatorTests {
  private Elevator elevator;
  private ElevatorCtrlSystem elevatorCtrl;

  @BeforeEach
  void setup() {
    elevatorCtrl = new ElevatorCtrlSystem();
    elevator = new Elevator(elevatorCtrl);
  }

  @Test
  void testMoveToPosition() {
    elevatorCtrl.setTargetPosition(ElevatorTarget.CORAL_L4);
    double totalTime = 10.0; // seconds
    double dt = 0.02; // 20ms loop
    int iterations = (int) (totalTime / dt);

    for (int i = 0; i < iterations; i++) {
      elevator.periodic();
    }

    assertTrue(elevatorCtrl.atTargetPosition());
  }
}
