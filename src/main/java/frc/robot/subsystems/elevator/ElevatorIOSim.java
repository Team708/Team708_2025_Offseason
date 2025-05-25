package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {
  private ElevatorSim elevatorSim;
  private DCMotor gearBox;
  private double appliedVolts;

  public ElevatorIOSim() {
    elevatorSim =
        new ElevatorSim(
            kMotors,
            kMotorReduction,
            kCarriageMassKg,
            kDrumRadiusMeters,
            kMinHeightMeters,
            kMaxHeightMeters,
            kSimulateGravity,
            kStartingHeightMeters);
    appliedVolts = 0.0;
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    elevatorSim.update(kSimUpdateInterval);
    inputs.motor1Connected = true;
    inputs.motor2Connected = true;
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = elevatorSim.getCurrentDrawAmps();
    inputs.positionMeters = elevatorSim.getPositionMeters();
    inputs.velocityMetersPerSecond = elevatorSim.getVelocityMetersPerSecond();
    inputs.rpm =
        (elevatorSim.getVelocityMetersPerSecond() * 60) / (2 * Math.PI * kDrumRadiusMeters);

    // Hard limits
    if (inputs.positionMeters <= 0 && inputs.velocityMetersPerSecond < 0) {
      elevatorSim.setState(VecBuilder.fill(0.0, 0.0));
    } else if (inputs.positionMeters >= kMaxHeightMeters && inputs.velocityMetersPerSecond > 0) {
      elevatorSim.setState(VecBuilder.fill(kMaxHeightMeters, 0.0));
    }
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = volts;
    elevatorSim.setInputVoltage(volts);
  }
}
