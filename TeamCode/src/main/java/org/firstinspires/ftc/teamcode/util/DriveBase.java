package org.firstinspires.ftc.teamcode.util;

import java.lang.Math;
import java.util.Optional;
import java.util.function.Consumer;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;

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

  public static double DEFAULT_POWER = 0.2;

  DcMotorEx leftFront;
  DcMotorEx leftRear;
  DcMotorEx rightFront;
  DcMotorEx rightRear;

  public DriveBase(HardwareMap hardwareMap) {
    leftFront = (DcMotorEx) hardwareMap.dcMotor.get("left_front");
    leftRear = (DcMotorEx) hardwareMap.dcMotor.get("left_rear");
    rightFront = (DcMotorEx) hardwareMap.dcMotor.get("right_front");
    rightRear = (DcMotorEx) hardwareMap.dcMotor.get("right_rear");
  }

  void eachMotor(Consumer<DcMotorEx> cb) {
    cb.accept(leftFront);
    cb.accept(leftRear);
    cb.accept(rightFront);
    cb.accept(rightRear);
  }

  public class Movement {
    public class Ticks {
      public int lf;
      public int lr;
      public int rf;
      public int rr;
      public int tolerance = 1;

      public Ticks(int lf, int lr, int rf, int rr) {
        this.lf = lf;
        this.lr = lr;
        this.rf = rf;
        this.rr = rr;
      }

      public Ticks relativeTo(DriveBase driveBase) {
        Ticks result = new Ticks(driveBase.leftFront.currentPosition + lf, driveBase.leftRear.currentPosition + lr,
            driveBase.rightFront.currentPosition + rf, driveBase.rightRear.currentPosition + rr);
        result.tolerance = tolerance;
      }

      public void apply(DriveBase driveBase) {
        driveBase.leftFront.setTargetPosition(lf);
        driveBase.leftRear.setTargetPosition(lr);
        driveBase.rightFront.setTargetPosition(rf);
        driveBase.rightRear.setTargetPosition(rr);

        driveBase.eachMotor((motor) -> motor.setTargetPositionTolerance(tolerance));
      }
    }

    Optional<Integer> power = Optional.empty();
    Optional<Ticks> ticks = Optional.empty();

    public void run(DriveBase driveBase) {
      if (ticks.isPresent()) {
        driveBase.eachMotor((motor) -> motor.setMode(RunMode.RUN_USING_ENCODER));

        Ticks relativeTicks = ticks.get().relativeTo(driveBase);
        relativeTicks.apply(driveBase);

        driveBase.eachMotor((motor) -> motor.setMode(RunMode.RUN_TO_POSITION));
      }

      if (power.isPresent())
        driveBase.eachMotor((motor) -> motor.setPower(power.get()));
    }

    public class Builder {
      Movement movement = new Movement();

      public Builder() {
      }

      public Builder withPower(int power) {
        movement.power = Optional.of(power);
        return this;
      }

      public Builder withTicks(int lf, int lr, int rf, int rr) {
        movement.ticks = Optional.of(new Ticks(lf, lr, rf, rr));
        return this;
      }

      public Builder withTolerance(int tolerance) {
        if (movement.ticks.isPresent()) {
          movement.ticks.get().tolerance = tolerance;
        }
      }

      public Movement build() {
        return movement;
      }
    }
  }
}
