package frc.robot.field;

import frc.robot.field.FieldConstants.AprilTagStruct;
import java.util.List;

public class FieldUtils {
  public static class Reef {
    public final AprilTagStruct tag;

    public Reef(AprilTagStruct tag) {
      this.tag = tag;
    }
  }

  public static List<AprilTagStruct> getReefTags() {
    return List.of();
  }

  public static Reef getClosestReef() {
    return new Reef(new AprilTagStruct(0));
  }
}
