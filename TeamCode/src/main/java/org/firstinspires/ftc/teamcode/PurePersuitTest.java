package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.List;

//for futur reference, Math is intrinsically imported

class VectorMath {

    static int n = 2;//how many places there are in our points

    //adds two points together
    static double[] add(double[] pt1, double[] pt2){
        double[] addition = {pt1[0] + pt2[0], pt1[1] + pt2[1]};
        return addition;
    }

    //Subtracts a point from another point (technically to get a Vector)
    static double[] subtract(double[] pt1, double[] pt2){
        double[] vect = {pt1[0]-pt2[0],pt1[1]-pt2[1]};
        return vect;
    }

    //multiply a point/vector by a constant
    static double[] multiply(double[] pt, double constant){
        double[] product = {pt[0] *constant, pt[1] *constant};
        return product;
    }

    // Function that return
    // dot product of two vector array.
    static double dotProduct(double vect_A[], double vect_B[]) {

        double product = 0;

        // Loop for calculate cot product
        for (int i = 0; i < n; i++)
            product = product + vect_A[i] * vect_B[i];
        return product;
    }

    // Function to find
    // cross product of two vector array.       //Don't think we can use or need below, we're only handling 2D points, also currently handles types incrrectly
    static int[] crossProduct(int vect_A[], int vect_B[]) {
        int[]  cross_P = new int[3];

        cross_P[0] = vect_A[1] * vect_B[2]
                - vect_A[2] * vect_B[1];
        cross_P[1] = vect_A[0] * vect_B[2]
                - vect_A[2] * vect_B[0];
        cross_P[2] = vect_A[0] * vect_B[1]
                - vect_A[1] * vect_B[0];
        return cross_P;
    }

    //Finds the distance between two points A and B
    static double distance(double[] pt_A, double[] pt_B){
        double distance = Math.sqrt(Math.pow(pt_B[1] - pt_A[1],2) + Math.pow(pt_B[0] - pt_A[0],2));
        return distance;
    }

    //find the curvature of the curve that intersects the three points
    static double curvature(double[] pt1, double[] pt2, double[] pt3){
        pt1[0] += 0.001; //prevents a divide-by-zero error that happens in pt1[0] == pt2[0]
        double k1 = 0.5*(Math.pow(pt1[0], 2) + Math.pow(pt1[1], 2) - Math.pow(pt2[0], 2) - Math.pow(pt2[1], 2))/(pt1[0]-pt2[0]);
        double k2 = (pt1[1] - pt2[1])/(pt1[0]-pt2[0]);
        double b = 0.5 *(Math.pow(pt2[0], 2) - 2*pt2[0]*k1 + Math.pow(pt2[1], 2) - Math.pow(pt3[0], 2) + 2*pt3[0]*k1 - Math.pow(pt3[1], 2))/(pt3[0]*k2-pt3[1]+pt2[1]-pt2[0]*k2);
        double a = k1 - k2*b;
        double r = Math.sqrt(Math.pow(pt1[0]-a, 2)+Math.pow(pt1[1]-b, 2)); //radius of the circle
        //if the curvature solves to NaN, the slope is 0 - a straight line
        if(1/r == Double.NaN){
            return 0;
        }
        else{
            return 1/r;
        }
    }

    //limit the power at any point to between max power and a constant/curvature
    static double clipVelocity(double power, int k, double robotPower, double curvature){
        //power is our calculated power for the turn
        //k is some constant between 1 and 5 that determines how fast we go around corners
        //robotPower is the public power variable
        //curvature is the paths curvature at that point
        return Math.min(robotPower, Math.max(k/curvature, Math.min(power, robotPower)));
    }

    //uses math to find where a circle intersects a vector
    //used to find the robot's lookahead point
    static double[] intersectionPoint(double[] point, double[] nextPoint, double[] circleCenter, double circleRadius){
        //bunch of functions below ostensibly don't need VectorMath in front of them because they're in the same class as this function
        //  -IDE says it should work, but might be wrong
        double t = -1.0;
        double[] lookaheadPoint = new double[2];//should intialize as {null, null}

        double[] d = subtract(nextPoint,point);//not sure how this works is the closest point is the end of the path
        double[] f = subtract(point, circleCenter);

        double a = dotProduct(d,d);
        double b = dotProduct(multiply(f, 2.0), d);
        double c = dotProduct(f,f) - circleRadius*circleRadius;//Math.pow doesn't work for some reason
        double discriminant = b*b - 4*a*c;

        if (discriminant < 0){
            //no intersection
            //which means the robot can't 'see'  the vector it needs to
        }
        else{
            discriminant = Math.sqrt(discriminant);
            double t1 = (-b - discriminant)/(2*a);
            double t2 = (-b + discriminant)/(2*a);

            if(t1 >= 0 && t1 <= 1){
                t = t1;
            }
            else if (t2 >= 0 && t2 <= 1){
                t = t2;
            }

            //otherwise, no intersection

        }

        if(t != -1.0) {
            return add(point, multiply(d, t));//calculate point at which the circle intersects the vector
        }
        else{ return null; }//the circle does not intersect the vector
    }


}

@Autonomous(name="Pure Persuit", group="Linear Opmode")
@Disabled
public class PurePersuitTest extends LinearOpMode{
    //To Do:
    //  -Skipped parts 2-Acceleration Limiter and 3-Calculations in velocities for now
    //      -I maybe should be calculating the clipVelocity() of each point alongside dists and curves
    //  -Skipped Controlling Wheel Velocities and tuning
    //  -Combine path generation into a single loop? Do all the calculations for each variable for each point simultaneously?

    //location properties of the robot, later we should make sure the robot can start from a place other than [0,0]
    //As it is now, coordinates are relative to the robots starting position, which is always [0,0] despite where it actually is on the field
    public double[] robotXY = {0,0};
    public double robotZ = 0; //rotation around the z-axis of the robot in radians
    public double robotWidth = 14;//width of the robot, should be a few inches more than actual
    public int robotDist = 0;
    public int robotLookDist = 103;   //too small value might not work? Affects how 'smooth' the route it takes is, see tutorial
                                    //also, remember this is in encoder counts, currently at ~1/10 of a tile
    double power = 0.5;

    DcMotor leftMotor;
    DcMotor rightMotor;
    BNO055IMU imu;

    @Override
    public void runOpMode() {
        //will need to adjust to work with 4 wheels, pray this doesn't screw up the math
        rightMotor = hardwareMap.dcMotor.get("right_motor");
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);//we use encoders but not run to position and don't need PID to power
        //
        leftMotor = hardwareMap.dcMotor.get("left_motor");//hardwareMap.get(DcMotor.class, "left_motor");
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        while(!imu.isGyroCalibrated()){
            //I don't know how necessary this is, decided to include it
        }

        waitForStart();

        //path of points we're following
        //coordinates should currently map to encoder counts, i.e. [1,1] is 1 encoder count going diagonally up and right
        //an encoder ocunt varies based on the circumfrence of the weel
        //field tiles are 22.75"(577.9mm) by 22.75"
        //(200*PI*Encoder Count)/1120 = distance travelled in mm
        //So [1030 , 1030] would make it travel one tile distance diagonally up and right
        //Above ignores the gear ratios on current robot we're testing, which is a 60T-90T ratio
        double[][] path = {{1030,1030}};

        //make a list with the distances of each point along the entire path
        double[] pathDists = new double[path.length];
        pathDists[0] = 0; //the first point will always be at distance 0 along the line
        for(int i = 1; i < path.length; i++){
            pathDists[i] = pathDists[i-1] + VectorMath.distance(path[i-1], path[i]);
        }

        //Make a list of the curvatures at all the points
        double[] pathCurves = new double[path.length];
        pathCurves[0] = 0;      //First and last points on the path don't have 2 adjacent points, so don't have a curvature
        pathCurves[pathCurves.length - 1] = 0;
        for(int i = 1; i < path.length-1; i++){
            pathCurves[i] = VectorMath.curvature(path[i], path[i-1], path[i+1]);
        }

        //math require finding the change in the encoder count, cur EncoderPos - prev EncoderPos
        int lastRightEncoderCount = 0;
        int lastLeftEncoderCount = 0;
        double[] lookaheadPoint = new double[2];

        while(opModeIsActive()){
            /*

            //update the robot's position values (odometry), average recorded values w/ previous ones
            //Note: Future odometry is ideally handled by odometry wheels, can start working on code w/ imu.getAcceleration()
            robotZ = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
            robotDist = (leftMotor.getCurrentPosition() - lastLeftEncoderCount + rightMotor.getCurrentPosition() - lastRightEncoderCount)/2;
            robotXY[0] += robotDist * Math.cos(robotZ);
            robotXY[1] += robotDist * Math.sin(robotZ);

            lastRightEncoderCount = rightMotor.getCurrentPosition();
            lastLeftEncoderCount = leftMotor.getCurrentPosition();

            //find the iterator of the closest point in path
            //possibly have a problem where it chooses a point behind the robot
            //this doesn't take into account that we might be ahead of or behind the closest point with no way of telling which
            //  -meaning we could be on or entirely behind the line between the closest and look-ahead point
            //  -the tutorial doesn't mention this, so for now I'm assuming it isn't a problem
            //  -I guess this should only break if our look ahead distance (robotLookDist) is too small
            int closestIter = 0;
            //we start from the current point and iterate up the order of points in path
            for(int i = closestIter; i<path.length; i++){
                //might have to add && distance_to_point = robotLookDist
                if(VectorMath.distance(robotXY,path[i]) < VectorMath.distance(robotXY,path[closestIter])){
                    closestIter = i;
                }
            }

            //find the lookahead point
            //we don't check point path.length-1 because that's the endpoint in the path and doesn't have a point afterwards to make a vector with
            for(int i = closestIter; i<path.length-1; i++){
                double[] curLookaheadPoint = VectorMath.intersectionPoint(path[i], path[i+1], robotXY, robotLookDist);
                if(curLookaheadPoint != null){
                    lookaheadPoint = curLookaheadPoint;//if we can see a new lookahead point, we assign it, otherwise we stick with the last one
                }
            }

            //Find the curvature of the arc between the robot's position and the lookaheadPoint
            double curvature = 2*lookaheadPoint[0]/Math.pow(robotLookDist, 2);
            //find what side the robot is on (helps find which way it needs to turn)
            double side = Math.signum(Math.sin(robotZ) * (lookaheadPoint[0]-robotXY[0]) - Math.cos(robotZ)*(lookaheadPoint[1]-robotXY[1]));

            double signedCurvature = side*curvature;

            //find the velocities for the wheels
            //calculation below are for a tank-drive robot, our four wheel design will differ
            double leftVelocity = VectorMath.clipVelocity(power*(2+curvature*robotWidth)/2, 5, power, curvature);//clipping may or may not work correctly
            double rightVelocity = VectorMath.clipVelocity(power*(2-curvature*robotWidth)/2, 5, power, curvature);

            rightMotor.setPower(rightVelocity);
            leftMotor.setPower(leftVelocity);
            /*
            rightMotor.setTargetPosition(100);
            leftMotor.setTargetPosition(100);
            */
            //rightMotor.setTargetPosition(1000);
            //leftMotor.setTargetPosition(1000);

            rightMotor.setPower(power);
            leftMotor.setPower(power);

            //rightMotor.

            //see the tutorial's notes on stopping
            telemetry.clear();
            telemetry.addData("Right Encoder:", rightMotor.getCurrentPosition());
            telemetry.addData("Left Encoder:", leftMotor.getCurrentPosition());
            telemetry.update();
        }

    }
}
