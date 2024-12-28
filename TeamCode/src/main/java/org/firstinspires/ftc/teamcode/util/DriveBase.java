package org.firstinspires.ftc.teamcode.util;

import java.lang.Math;
import java.util.Optional;
import java.util.function.Consumer;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;

public class DriveBase {
  public static double wheelDiameterMM = 96.0;
  public static double wheelRadiusMM = wheelDiameterMM / 2.0;
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

  public static class Movement {
    public static class PowerLevels implements Consumer<DriveBase> {
      public double lf;
      public double lr;
      public double rf;
      public double rr;

      public PowerLevels(double lf, double lr, double rf, double rr) {
        this.lf = lf;
        this.lr = lr;
        this.rf = rf;
        this.rr = rr;
      }

      public void accept(DriveBase driveBase) {
        driveBase.leftFront.setPower(lf);
        driveBase.leftRear.setPower(lr);
        driveBase.rightFront.setPower(rf);
        driveBase.rightRear.setPower(rr);
      }
    }

    public static class Ticks implements Consumer<DriveBase> {
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
        Ticks result = new Ticks(driveBase.leftFront.getCurrentPosition() + lf,
            driveBase.leftRear.getCurrentPosition() + lr,
            driveBase.rightFront.getCurrentPosition() + rf, driveBase.rightRear.getCurrentPosition() + rr);
        result.tolerance = tolerance;
        return result;
      }

      public PowerLevels calculateEvenPower(double maxPower) {
        double maxTicks = Math.max(Math.max(lf, lr), Math.max(rf, rr));

        return new PowerLevels(
            (lf / maxTicks) * maxPower,
            (lr / maxTicks) * maxPower,
            (rf / maxTicks) * maxPower,
            (rr / maxTicks) * maxPower);
      }

      public void accept(DriveBase driveBase) {
        driveBase.leftFront.setTargetPosition(lf);
        driveBase.leftRear.setTargetPosition(lr);
        driveBase.rightFront.setTargetPosition(rf);
        driveBase.rightRear.setTargetPosition(rr);

        driveBase.eachMotor((motor) -> motor.setTargetPositionTolerance(tolerance));
      }
    }

    Optional<PowerLevels> power = Optional.empty();
    Optional<Ticks> ticks = Optional.empty();

    public void run(DriveBase driveBase) {
      if (ticks.isPresent()) {
        driveBase.eachMotor((motor) -> motor.setMode(RunMode.RUN_USING_ENCODER));

        Ticks relativeTicks = ticks.get().relativeTo(driveBase);
        relativeTicks.accept(driveBase);
        driveBase.eachMotor((motor) -> motor.setMode(RunMode.RUN_TO_POSITION));
      }

      if (power.isPresent())
        power.get().accept(driveBase);
    }

    public static class Builder {
      Movement movement = new Movement();

      public Builder() {
      }

      public Builder withPower(PowerLevels powerLevels) {
        movement.power = Optional.ofNullable(powerLevels);
        return this;
      }

      public Builder withPower(double power) {
        return withPower(new PowerLevels(power, power, power, power));
      }

      public Builder withTicks(Ticks ticks) {
        movement.ticks = Optional.ofNullable(ticks);
        return this;
      }

      public Builder withTicks(int lf, int lr, int rf, int rr) {
        return withTicks(new Ticks(lf, lr, rf, rr));
      }

      public Builder withTolerance(int tolerance) {
        if (movement.ticks.isPresent()) {
          movement.ticks.get().tolerance = tolerance;
        }
        return this;
      }

      public Builder withPolar(double rho, double theta, double maxPower) {
        PolarSegment segment = new PolarSegment(rho, theta);
        Ticks ticks = segment.calculateTicks();

        return this.withTicks(ticks).withPower(ticks.calculateEvenPower(maxPower));
      }

      /**
       * @see Builder#withPolar(double, double, int)
       *      Like withPolar(), but using millimeters for rho instead of ticks
       */
      public Builder withPolarMM(double rho, double theta, double maxPower) {
        return this.withPolar(rho * motorEncoderEventsPerMM, theta, maxPower);
      }

      public Movement build() {
        return movement;
      }
    }

    public static Builder builder() {
      return new Builder();
    }

    public static class PolarSegment {
      /** radius (in ticks) */
      double rho;

      /** angle (in radians) */
      double theta;

      /**
       * @param rho   radius (ticks)
       * @param theta angle (radians)
       */
      public PolarSegment(double rho, double theta) {
        this.rho = rho;
        this.theta = theta;
      }

      public Ticks calculateTicks() {
        int lf = (int) (rho * (Math.cos(theta) - Math.sin(theta)));
        int lr = (int) (rho * (Math.cos(theta) + Math.sin(theta)));
        int rr = lf;
        int rf = lr;

        return new Ticks(lf, lr, rf, rr);
      }
    }
  }

  public void runPolar(double rho, double theta, double maxPower) {
    Movement
        .builder()
        .withPolar(rho, theta, maxPower)
        .build()
        .run(this);
  }

  public void runPolarMM(double rho, double theta, double maxPower) {
    Movement
        .builder()
        .withPolarMM(rho, theta, maxPower)
        .build()
        .run(this);
  }
}
