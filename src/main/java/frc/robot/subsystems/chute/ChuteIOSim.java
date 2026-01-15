package frc.robot.subsystems.chute;

import static frc.robot.subsystems.chute.ChuteConstants.*;

import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;
import frc.robot.util.UnitUtil;

public class ChuteIOSim implements ChuteIO {
  private final ElevatorSim chuteSim;
  private double appliedVolts;

  public ChuteIOSim() {
    chuteSim =
        new ElevatorSim(
            Constants.K1_VORTEX,
            MOTOR_REDUCTION,
            UnitUtil.poundsToKilograms(MASS_LBS),
            EFFECTIVE_RADIUS,
            UnitUtil.inchesToMeters(RETRACTED_INCHES),
            UnitUtil.inchesToMeters(EXTENDED_INCHES),
            false,
            UnitUtil.inchesToMeters(RETRACTED_INCHES));
    appliedVolts = 0.0;
  }

  @Override
  public void updateInputs(ChuteIOInputs inputs) {
    chuteSim.update(SIM_UPDATE_INTERVAL);
    inputs.connected = true;
    inputs.positionInches = UnitUtil.metersToInches(chuteSim.getPositionMeters());
    inputs.isFullyRetracted = inputs.positionInches <= RETRACTED_INCHES + TOLERANCE;
    inputs.isFullyExtended = inputs.positionInches >= EXTENDED_INCHES - TOLERANCE;
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = chuteSim.getCurrentDrawAmps();
    inputs.velocityInchesPerSecond = UnitUtil.metersToInches(chuteSim.getVelocityMetersPerSecond());
    inputs.rpm = (inputs.velocityInchesPerSecond * 60) / SCREW_INCHES_PER_REV;

    if (inputs.isFullyRetracted && inputs.velocityInchesPerSecond < 0) {
      chuteSim.setState(UnitUtil.inchesToMeters(RETRACTED_INCHES), 0.0);
    } else if (inputs.isFullyExtended && inputs.velocityInchesPerSecond > 0) {
      chuteSim.setState(UnitUtil.inchesToMeters(EXTENDED_INCHES), 0.0);
    }
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = volts;
    chuteSim.setInputVoltage(volts);
  }
}
