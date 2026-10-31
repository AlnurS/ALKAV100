package org.firstinspires.ftc.teamcode.pedroPathing.pathgen;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Represents a Bezier curve path segment defined by control points.
 */
public class BezierCurve {
    private List<Point> controlPoints;

    public BezierCurve(Point... points) {
        this.controlPoints = new ArrayList<>(Arrays.asList(points));
    }

    public List<Point> getControlPoints() {
        return controlPoints;
    }

    public Point getStartPoint() {
        return controlPoints.get(0);
    }

    public Point getEndPoint() {
        return controlPoints.get(controlPoints.size() - 1);
    }

    /**
     * Calculates the point P(t) on the Bezier curve for progress t in [0.0, 1.0]
     * using de Casteljau's algorithm.
     */
    public Point getPoint(double t) {
        if (controlPoints.isEmpty()) return new Point(0, 0);
        if (controlPoints.size() == 1) return controlPoints.get(0);

        List<Point> current = new ArrayList<>(controlPoints);
        while (current.size() > 1) {
            List<Point> next = new ArrayList<>();
            for (int i = 0; i < current.size() - 1; i++) {
                double x = (1 - t) * current.get(i).getX() + t * current.get(i + 1).getX();
                double y = (1 - t) * current.get(i).getY() + t * current.get(i + 1).getY();
                next.add(new Point(x, y));
            }
            current = next;
        }
        return current.get(0);
    }
}
