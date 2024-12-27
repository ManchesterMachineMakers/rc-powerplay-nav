package org.firstinspires.ftc.teamcode.util;

import java.lang.Math;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class DriveBase {
  public static double wheelDiameterMM = 96.0;
  public static double wheelRotationDistanceMM = Math.PI * wheelDiameterMM;
  public static double motorRotationsPerMM = 1 / wheelRotationDistanceMM;

  // English
  public static double wheelDiameterInches = Conversions.mmToInches(wheelDiameterMM);
  public static double wheelRotationDistanceInches = Math.PI * wheelDiameterInches;
  public static double motorRotationsPerInch = 1 / wheelRotationDistanceInches;

  public static double wheelBaseWidth = 16.0; // inches

  public static double wheelBaseLengthMM = 360.0;
  public static double inchwormFrontMM = 216.0; // front to pivot point

  public static double inchwormRearMM = 144.0; // rear to pivot point

  // https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/
  public static double motorEncoderEventsPerRotation = 537.7;
  public static double motorEncoderEventsPerInch = motorRotationsPerInch * motorEncoderEventsPerRotation;
  public static double motorEncoderEventsPerMM = motorRotationsPerMM * motorEncoderEventsPerRotation;

  private DcMotorEx leftFront;
  private DcMotorEx leftRear;
  private DcMotorEx rightFront;
  private DcMotorEx rightRear;

  public DriveBase(HardwareMap hardwareMap) {
    leftFront = (DcMotorEx) hardwareMap.dcMotor.get("left_front");
    leftRear = (DcMotorEx) hardwareMap.dcMotor.get("left_rear");
    rightFront = (DcMotorEx) hardwareMap.dcMotor.get("right_front");
    rightRear = (DcMotorEx) hardwareMap.dcMotor.get("right_rear");
  }
}
