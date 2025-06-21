package frc.robot.subsystems.chute;

import static frc.robot.subsystems.chute.ChuteConstants.*;

import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;
import frc.robot.util.UnitUtil;

public class ChuteIOSim implements IChuteIO {
  private final ElevatorSim chuteSim;
  private double appliedVolts;

  public ChuteIOSim() {
    chuteSim =
        new ElevatorSim(
            Constants.k1Vortex,
            kMotorReduction,
            UnitUtil.poundsToKilograms(kMassLbs),
            kEffectiveRadius,
            UnitUtil.inchesToMeters(kRetractedInches),
            UnitUtil.inchesToMeters(kExtendedInches),
            false,
            UnitUtil.inchesToMeters(kRetractedInches));
    appliedVolts = 0.0;
  }

  @Override
  public void updateInputs(ChuteIOInputs inputs) {
    chuteSim.update(kSimUpdateInterval);
    inputs.connected = true;
    inputs.positionInches = UnitUtil.metersToInches(chuteSim.getPositionMeters());
    inputs.isFullyRetracted = inputs.positionInches <= kRetractedInches + kTolerance;
    inputs.isFullyExtended = inputs.positionInches >= kExtendedInches - kTolerance;
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = chuteSim.getCurrentDrawAmps();
    inputs.velocityInchesPerSecond = UnitUtil.metersToInches(chuteSim.getVelocityMetersPerSecond());
    inputs.rpm = (inputs.velocityInchesPerSecond * 60) / kScrewInchesPerRev;

    if (inputs.isFullyRetracted && inputs.velocityInchesPerSecond < 0) {
      chuteSim.setState(UnitUtil.inchesToMeters(kRetractedInches), 0.0);
    } else if (inputs.isFullyExtended && inputs.velocityInchesPerSecond > 0) {
      chuteSim.setState(UnitUtil.inchesToMeters(kExtendedInches), 0.0);
    }
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = volts;
    chuteSim.setInputVoltage(volts);
  }
}
