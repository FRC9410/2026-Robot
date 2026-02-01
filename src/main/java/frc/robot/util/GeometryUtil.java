// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;

public final class GeometryUtil {
  private GeometryUtil() {}

  /**
   * Returns true if point p is inside polygon poly (or on its boundary). poly: vertices in order
   * (clockwise or ccw), not necessarily closed.
   */
  public static boolean pointInPolygon(List<Translation2d> poly, Translation2d p) {
    int n = poly.size();
    if (n < 3) return false;

    // Treat "on an edge" as inside
    for (int i = 0, j = n - 1; i < n; j = i++) {
      if (pointOnSegment(poly.get(j), poly.get(i), p, 1e-9)) return true;
    }

    boolean inside = false;
    for (int i = 0, j = n - 1; i < n; j = i++) {
      double xi = poly.get(i).getX(), yi = poly.get(i).getY();
      double xj = poly.get(j).getX(), yj = poly.get(j).getY();
      boolean intersect =
          ((yi > p.getY()) != (yj > p.getY()))
              && (p.getX()
                  < (xj - xi) * (p.getY() - yi) / ((yj - yi) == 0 ? 1e-12 : (yj - yi)) + xi);
      if (intersect) inside = !inside;
    }
    return inside;
  }

  private static boolean pointOnSegment(
      Translation2d a, Translation2d b, Translation2d p, double eps) {
    double ax = a.getX(), ay = a.getY();
    double bx = b.getX(), by = b.getY();
    double px = p.getX(), py = p.getY();

    double abx = bx - ax, aby = by - ay;
    double apx = px - ax, apy = py - ay;

    double cross = abx * apy - aby * apx;
    if (Math.abs(cross) > eps) return false;

    double dot = abx * apx + aby * apy;
    if (dot < -eps) return false;

    double ab2 = abx * abx + aby * aby;
    if (dot > ab2 + eps) return false;

    return true;
  }
}
