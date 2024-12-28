package org.firstinspires.ftc.teamcode.util;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class SegmentTest {

  @Test
  public void ticksStraight() {
    assertEquals(100, new DriveBase.Movement.PolarSegment(100, 0).calculateTicks().lf, 0.01);
  }

}
