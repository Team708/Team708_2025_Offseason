package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;
import frc.robot.util.UnitUtil;

public class ElevatorIOSim implements ElevatorIO {
  private PIDController controller;
  private ElevatorSim elevatorSim;
  private double appliedVolts;

  public ElevatorIOSim() {
    elevatorSim =
        new ElevatorSim(
            Constants.k2Vortex,
            kMotorReduction,
            UnitUtil.poundsToKilograms(kCarriageMassLbs),
            UnitUtil.inchesToMeters(kEffectiveDrumRadiusInches),
            UnitUtil.inchesToMeters(kMinHeightInches),
            UnitUtil.inchesToMeters(kMaxHeightInches),
            kSimulateGravity,
            UnitUtil.inchesToMeters(kStartingHeightInches));
    appliedVolts = 0.0;
    controller = new PIDController(kP, kI, kD);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    elevatorSim.update(kSimUpdateInterval);
    double setPoint = controller.calculate(inputs.positionInches, inputs.targetInches);
    elevatorSim.setInputVoltage(MathUtil.clamp(setPoint, -kMaxVoltage, kMaxVoltage));
    inputs.motor1Connected = true;
    inputs.motor2Connected = true;
    inputs.appliedVolts = MathUtil.clamp(setPoint, -kMaxVoltage, kMaxVoltage);
    inputs.currentAmps = elevatorSim.getCurrentDrawAmps();
    inputs.positionInches = UnitUtil.metersToInches(elevatorSim.getPositionMeters());
    inputs.velocityInchesPerSecond =
        UnitUtil.metersToInches(elevatorSim.getVelocityMetersPerSecond());
    inputs.rpm =
        (UnitUtil.metersToInches(elevatorSim.getVelocityMetersPerSecond()) * 60 * kMotorReduction)
            / (2 * Math.PI * kEffectiveDrumRadiusInches);

    // Hard limits
    if (inputs.positionInches <= 0 && inputs.velocityInchesPerSecond < 0) {
      elevatorSim.setState(VecBuilder.fill(0.0, 0.0));
    } else if (inputs.positionInches >= kMaxHeightInches && inputs.velocityInchesPerSecond > 0) {
      elevatorSim.setState(VecBuilder.fill(kMaxHeightInches, 0.0));
    }

    // Zero triggered
    if (inputs.positionInches <= 0) {
      inputs.reverseLimitTriggered = true;
    } else {
      inputs.reverseLimitTriggered = false;
    }
  }
}
