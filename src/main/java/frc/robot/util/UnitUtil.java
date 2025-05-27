package frc.robot.util;

public class UnitUtil {
  public static double kilogramsToPounds(double kg) {
    return kg * 2.20462;
  }

  public static double poundsToKilograms(double pounds) {
    return pounds / 2.20462;
  }

  public static double metersToInches(double meters) {
    return meters * 39.3701;
  }

  public static double inchesToMeters(double inches) {
    return inches / 39.3701;
  }
}
