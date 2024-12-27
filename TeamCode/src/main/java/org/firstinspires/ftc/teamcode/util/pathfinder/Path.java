package org.firstinspires.ftc.teamcode.util.pathfinder;

import org.firstinspires.ftc.teamcode.util.DriveBase;

public class Segment {
  int xTicks;
  int yTicks;

  public Segment(int x, int y) {
    xTicks = x * DriveBase.motorEncoderEventsPerMM;
    yTicks = y * DriveBase.motorEncoderEventsPerMM;
  }
}
