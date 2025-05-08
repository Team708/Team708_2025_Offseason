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
    gearBox = DCMotor.getNEO(1);
    linearSystem = LinearSystemId.createDCMotorSystem(gearBox, 0.025, kMotorReduction);
    motorSim = new DCMotorSim(linearSystem, gearBox);
    appliedVolts = 0.0;
  }

  @Override
  public void updateInputs(ChuteIOInputs inputs) {
    motorSim.update(kSimUpdateInterval);
    inputs.connected = true;
    inputs.isFullyRetracted = inputs.positionMeters <= kRetractedMeters;
    inputs.isFullyExtended = inputs.positionMeters >= kExtendedMeters;
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = motorSim.getCurrentDrawAmps();
    inputs.positionMeters = motorSim.getAngularPositionRotations() * kScrewTravelPerRev;
    inputs.velocityMetersPerSecond = motorSim.getAngularVelocityRPM() / 60.0 * kScrewTravelPerRev;

    if (inputs.positionMeters <= kRetractedMeters && inputs.velocityMetersPerSecond < 0) {
      System.out.println("HARD STOP RETRACT");
      // Hit bottom hard stop
      motorSim.setInput(0.0);
      motorSim.setState(VecBuilder.fill(kRetractedMeters, 0.0));
    } else if (inputs.positionMeters >= kExtendedMeters && inputs.velocityMetersPerSecond > 0) {
      System.out.println("HARD STOP EXTEND");
      // Hit top hard stop
      motorSim.setInput(0.0);
      motorSim.setState(VecBuilder.fill(kExtendedMeters * 2 * Math.PI, 0.0));
    }
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = volts;
    motorSim.setInputVoltage(volts);
  }
}
