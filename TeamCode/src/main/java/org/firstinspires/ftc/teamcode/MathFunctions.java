package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

//import org.opencv.core.Point;

import java.util.ArrayList;

import static java.lang.Math.pow;
import static java.lang.Math.sqrt;

@Disabled
public class MathFunctions {
    /**
     * AngleWrap makes sure the angle the robot is at is between -180 and 180 ( - PI to PI)
     * @param angle
     * @return
     */
    public static double AngleWrap(double angle){
        while(angle < -Math.PI){
            angle += 2 * Math.PI;
        }
        while(angle > Math.PI){
            angle -= 2 * Math.PI;
        }
        return angle;
    }

    /**
     * This Array List draws a circle and where that circle intersects a line (drawn by the target of the robot designated by teamcode.RobotMovement.goToPosition) it marks a point.
     * @param circleCenter
     * @param radius
     * @param linePoint1
     * @param linePoint2
     * @return
     */
    /*
    public static ArrayList<Point> lineCircleIntersection (Point circleCenter, double radius, Point linePoint1, Point linePoint2){

        //This avoids vertical and horizontal lines by eliminating any slopes that near 0 or infinity
        if(Math.abs(linePoint1.y - linePoint2.y) < .003){
            linePoint1.y = linePoint2.y + .003;
        }
        if(Math.abs(linePoint1.x - linePoint2.x) < .003){
            linePoint1.x = linePoint2.x + .003;
        }

        //Rise over Run to get slope of line
        double m1 = (linePoint2.y - linePoint1.y)/(linePoint2.x - linePoint1.x);

        //Math to set up quadratic formula
        double quadraticA = 1.0 + Math.pow(m1,2);

        double x1 = linePoint1.x - circleCenter.x;
        double y1 = linePoint1.y - circleCenter.y;

        double quadraticB = (2.0 * m1 * y1) - (2.0 * Math.pow(m1,2) * x1);

        double quadraticC = ((Math.pow(m1,2) * Math.pow(x1,2))) - (2.0*y1*m1*x1) + Math.pow(y1,2)-Math.pow(radius,2);

        ArrayList<Point> allPoints = new ArrayList<>();

        try{
            double xRoot1 = (-quadraticB + sqrt(pow(quadraticB,2) - (4.0 * quadraticA * quadraticC)))/(2.0 * quadraticA);

            double yRoot1 = m1 * (xRoot1 - x1) + y1;

            //Put back the offset
            xRoot1 += circleCenter.x;
            yRoot1 += circleCenter.y;

            double minX = linePoint1.x < linePoint2.x ? linePoint1.x : linePoint2.x;
            double maxX = linePoint1.x > linePoint2.x ? linePoint1.x : linePoint2.x;

            if(xRoot1 > minX && xRoot1 < maxX){
                allPoints.add(new Point(xRoot1,yRoot1));
            }

            double xRoot2 = (-quadraticB - sqrt(pow(quadraticB,2) - (4.0 * quadraticA * quadraticC)))/(2.0 * quadraticA);

            double yRoot2 = m1 * (xRoot2 - x1) + y1;

            xRoot2 += circleCenter.x;
            yRoot2 += circleCenter.y;

            if(xRoot1 > minX && xRoot1 < maxX){
                allPoints.add(new Point(xRoot2,yRoot2));
            }
        }catch(Exception e){

        }
        return allPoints;
    }*/
}
