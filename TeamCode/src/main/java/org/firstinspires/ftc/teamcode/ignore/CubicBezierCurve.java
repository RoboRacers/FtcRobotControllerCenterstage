package org.firstinspires.ftc.teamcode.ignore;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;


public class CubicBezierCurve extends path{
    private Vector2D P0, P1, P2, P3;  // Assume Position is a class representing a point with x and y coordinates


    public CubicBezierCurve(Vector2D P0, Vector2D P1, Vector2D P2, Vector2D P3) {
        this.P0 = P0;
        this.P1 = P1;
        this.P2 = P2;
        this.P3 = P3;
    }


    public Vector2D point(double t) {
        double x = bezierCurveX(t);
        double y = bezierCurveY(t);
        return new Vector2D(x, y);  // Assuming z-coordinate is 0 for simplicity
    }

    public void setP0(Vector2D p0) {
        P0 = p0;
    }

    public void setP1(Vector2D p1) {
        P1 = p1;
    }

    public void setP2(Vector2D p2) {
        P2 = p2;
    }

    public void setP3(Vector2D p3) {
        P3 = p3;
    }

    public Vector2D getP0() {
        return P0;
    }

    public Vector2D getP1() {
        return P1;
    }

    public Vector2D getP2() {
        return P2;
    }

    public Vector2D getP3() {
        return P3;
    }

    // Calculate x coordinate of Bezier curve at parameter t
    private double bezierCurveX(double t) {
        double term1 = Math.pow(1 - t, 3) * P0.getX();
        double term2 = 3 * Math.pow(1 - t, 2) * t * P1.getX();
        double term3 = 3 * (1 - t) * Math.pow(t, 2) * P2.getX();
        double term4 = Math.pow(t, 3) * P3.getX();
        return term1 + term2 + term3 + term4;
    }


    // Calculate y coordinate of Bezier curve at parameter t
    private double bezierCurveY(double t) {
        double term1 = Math.pow(1 - t, 3) * P0.getY();
        double term2 = 3 * Math.pow(1 - t, 2) * t * P1.getY();
        double term3 = 3 * (1 - t) * Math.pow(t, 2) * P2.getY();
        double term4 = Math.pow(t, 3) * P3.getY();
        return term1 + term2 + term3 + term4;
    }

}
