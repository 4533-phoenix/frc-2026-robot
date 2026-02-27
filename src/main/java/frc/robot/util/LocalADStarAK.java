// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Modified by FRC Team 4533 (Phoenix) 2026
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.util;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinder;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

// NOTE: This file is available at
// https://gist.github.com/mjansen4857/a8024b55eb427184dbd10ae8923bd57d

/**
 * AdvantageKit wrapper for the PathPlanner LocalADStar pathfinder.
 *
 * <p>This wrapper handles logging path data to AdvantageKit, allowing for recording, replaying, and
 * analyzing pathfinding results in simulation or via log files.
 */
public class LocalADStarAK implements Pathfinder {
  private final ADStarIO io = new ADStarIO();

  /**
   * Checks if a new path has been calculated since the last retrieval.
   *
   * @return True if a new path is available.
   */
  @Override
  public boolean isNewPathAvailable() {
    // Only update if not in replay mode
    if (!Logger.hasReplaySource()) {
      io.updateIsNewPathAvailable();
    }

    // Process inputs (handles logging when recording, reading when replaying)
    Logger.processInputs("LocalADStarAK", io);

    return io.isNewPathAvailable;
  }

  /**
   * Retrieves the most recently calculated path.
   *
   * @param constraints The path constraints to use when creating the path.
   * @param goalEndState The goal end state to use when creating the path.
   * @return The PathPlannerPath created from the points calculated by the pathfinder, or null.
   */
  @Override
  public PathPlannerPath getCurrentPath(PathConstraints constraints, GoalEndState goalEndState) {
    // Only update if not in replay mode
    if (!Logger.hasReplaySource()) {
      io.updateCurrentPathPoints(constraints, goalEndState);
    }

    // Process inputs (handles logging when recording, reading when replaying)
    Logger.processInputs("LocalADStarAK", io);

    if (io.currentPathPoints.isEmpty()) {
      return null;
    }

    return PathPlannerPath.fromPathPoints(io.currentPathPoints, constraints, goalEndState);
  }

  /**
   * Sets the start position to pathfind from.
   *
   * @param startPosition Start position on the field.
   */
  @Override
  public void setStartPosition(Translation2d startPosition) {
    if (!Logger.hasReplaySource()) {
      io.adStar.setStartPosition(startPosition);
    }
  }

  /**
   * Sets the goal position to pathfind to.
   *
   * @param goalPosition Goal position on the field.
   */
  @Override
  public void setGoalPosition(Translation2d goalPosition) {
    if (!Logger.hasReplaySource()) {
      io.adStar.setGoalPosition(goalPosition);
    }
  }

  /**
   * Sets the dynamic obstacles that should be avoided while pathfinding.
   *
   * @param obs A List of Translation2d pairs representing obstacles.
   * @param currentRobotPos The current position of the robot.
   */
  @Override
  public void setDynamicObstacles(
      List<Pair<Translation2d, Translation2d>> obs, Translation2d currentRobotPos) {
    if (!Logger.hasReplaySource()) {
      io.adStar.setDynamicObstacles(obs, currentRobotPos);
    }
  }

  /** IO implementation for logging ADStar data. */
  private static class ADStarIO implements LoggableInputs {
    public LocalADStar adStar = new LocalADStar();
    public boolean isNewPathAvailable = false;
    public List<PathPoint> currentPathPoints = Collections.emptyList();

    /**
     * Serializes the current path points into the log table.
     *
     * @param table The log table to write to.
     */
    @Override
    public void toLog(LogTable table) {
      table.put("IsNewPathAvailable", isNewPathAvailable);

      // Serialize path points to a double array (x, y pairs)
      double[] pointsLogged = new double[currentPathPoints.size() * 2];
      int idx = 0;
      for (PathPoint point : currentPathPoints) {
        pointsLogged[idx] = point.position.getX();
        pointsLogged[idx + 1] = point.position.getY();
        idx += 2;
      }

      table.put("CurrentPathPoints", pointsLogged);
    }

    /**
     * Deserializes path points from the log table.
     *
     * @param table The log table to read from.
     */
    @Override
    public void fromLog(LogTable table) {
      isNewPathAvailable = table.get("IsNewPathAvailable", false);

      // Deserialize double array back into path points
      double[] pointsLogged = table.get("CurrentPathPoints", new double[0]);

      List<PathPoint> pathPoints = new ArrayList<>();
      for (int i = 0; i < pointsLogged.length; i += 2) {
        pathPoints.add(
            new PathPoint(new Translation2d(pointsLogged[i], pointsLogged[i + 1]), null));
      }

      currentPathPoints = pathPoints;
    }

    /** Updates the flag indicating if a new path has been generated. */
    public void updateIsNewPathAvailable() {
      isNewPathAvailable = adStar.isNewPathAvailable();
    }

    /**
     * Updates the current path points based on the constraints and goal end state.
     *
     * @param constraints The path constraints.
     * @param goalEndState The goal end state.
     */
    public void updateCurrentPathPoints(PathConstraints constraints, GoalEndState goalEndState) {
      PathPlannerPath currentPath = adStar.getCurrentPath(constraints, goalEndState);

      if (currentPath != null) {
        currentPathPoints = currentPath.getAllPathPoints();
      } else {
        currentPathPoints = Collections.emptyList();
      }
    }
  }
}
