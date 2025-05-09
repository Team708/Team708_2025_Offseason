package frc.robot.subsystems.chute;

import static frc.robot.subsystems.chute.ChuteConstants.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ChuteIOSim implements ChuteIO {
  private final DCMotor gearBox;
  private final LinearSystem<N2, N1, N2> linearSystem;
  private final DCMotorSim motorSim;
  private double appliedVolts;

  public ChuteIOSim() {
    gearBox = DCMotor.getNEO(1).withReduction(kMotorReduction);
    linearSystem = LinearSystemId.createDCMotorSystem(gearBox, kJKgMetersSquared, kMotorReduction);
    motorSim = new DCMotorSim(linearSystem, gearBox);
    motorSim.setState(VecBuilder.fill((0.2 / kScrewTravelPerRev) * 2 * Math.PI, 0.0));
    appliedVolts = 0.0;
  }

  @Override
  public void updateInputs(ChuteIOInputs inputs) {
    motorSim.update(kSimUpdateInterval);
    inputs.connected = true;
    inputs.positionMeters = motorSim.getAngularPositionRotations() * kScrewTravelPerRev;
    inputs.isFullyRetracted = inputs.positionMeters <= kRetractedMeters + kTolerance;
    inputs.isFullyExtended = inputs.positionMeters >= kExtendedMeters - kTolerance;
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = motorSim.getCurrentDrawAmps();
    inputs.velocityMetersPerSecond = motorSim.getAngularVelocityRPM() / 60.0 * kScrewTravelPerRev;

    /*
     * Simulated hard stops
     * Set to min/max extension. Vecbuilder for state takes angularPosition and angularVelocity
     * In this context angularPosition is the total accumulated shaft rotation over time
     * Rotations = linear position / pitch
     * Angular position (radians) = rotations * 2PI
     */
    if (inputs.isFullyRetracted && inputs.velocityMetersPerSecond < 0) {
      motorSim.setState(VecBuilder.fill(kRetractedMeters, 0.0));
    } 
    else if (inputs.isFullyExtended && inputs.velocityMetersPerSecond > 0) {
      motorSim.setState(VecBuilder.fill((kExtendedMeters / kScrewTravelPerRev) * 2 * Math.PI, 0.0));
    }
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = volts;
    motorSim.setInputVoltage(volts);
  }
}
