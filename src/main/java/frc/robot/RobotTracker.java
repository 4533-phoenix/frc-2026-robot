package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.vision.PoseObservation;

public class RobotTracker {
  public static Pose2d getGlobalPose() {
    return new Pose2d();
  }

  public static void offerGlobalVisionObservation(PoseObservation observation) {}

  public static void offerConstrainedVisionObservation(PoseObservation observation) {}
}
