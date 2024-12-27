package org.firstinspires.ftc.teamcode.util;

public class Conversions {
  private static double mmPerInch = 25.4;

  public static double mmToInches(double mm) {
    return mm / mmPerInch;
  }

  public static double inchesToMM(double inches) {
    return inches * mmPerInch;
  }
}
