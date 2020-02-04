// Simple autonomous program that drives bot forward until end of period
// or touch sensor is hit. If touched, backs up a bit and turns 90 degrees
// right and keeps going. Demonstrates obstacle avoidance and use of the
// REV Hub's built in IMU in place of a gyro. Also uses gamepad1 buttons to
// simulate touch sensor press and supports left as well as right turn.
//
// Also uses PID controller to drive in a straight line when not
// avoiding an obstacle.
//
// Use PID controller to manage motor power during 90 degree turn to reduce
// overshoot.

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name="Foundation blue 15s", group="Foundation")
//@Disabled
public class OmniAutoFoundationBlue_15s extends LinearOpMode
{
    MiniPID controllerAngle = new MiniPID(0.035, 0, 0.03); //.025
    MiniPID controllerDrive = new MiniPID(0.035, 0, 0); //.025
    double globalAngle;


    public void stopDrive(){
        robot.frontleftDrive.setPower(0);
        robot.frontrightDrive.setPower(0);
        robot.backleftDrive.setPower(0);
        robot.backrightDrive.setPower(0);
    }

    //Past working values .035, 0, .03

    //Ziegler-Nichols standard for starting PID tuning values
    //Kcr = Proportional gain that causes steady osscillation (.04)
    //Pcr = Period of Kcr's Oscillation (measured in seconds) (1.4s) T
    //In a full PID system:
    //Proportional: .8Kcr
    //Derivative: (Ku*Tu)/10

    // called when init button is  pressed.

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right from zero point.
     */

    public void setAngle(){
        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = angles.firstAngle;
        globalAngle = deltaAngle;
    }

    public double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle;

        return -MathFunctions.AngleWrap(deltaAngle - globalAngle);
    }

    public void drivePIDtime(double power, double goalAngle, int direction, double time) {//-180 to 180
        double starTime = System.currentTimeMillis();
        controllerDrive.setOutputLimits(-1,1);
        while (true) {
            double correction = controllerDrive.getOutput(getAngle(), goalAngle);
            double y = -direction * power;
            double x = 0;
            double z = correction;
            robot.frontleftDrive.setPower(y + x + z);
            robot.frontrightDrive.setPower(-y + x + z);
            robot.backleftDrive.setPower(y - x + z);
            robot.backrightDrive.setPower(-y - x + z);
            if(System.currentTimeMillis()-starTime >= time) {
                stopDrive();
                break;
            }
            if(isStopRequested() == true){
                stopDrive();
                stop();
                break;
            }
        }
    }

    public double z_angle;

    OmniHardwarePushbot robot = new OmniHardwarePushbot();


    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException
    {

        robot.init(hardwareMap);
        waitForStart();
        sleep(10000);
        setAngle();

        robot.dragDriveRight.setPosition(robot.DDRI);
        robot.dragDriveLeft.setPosition(robot.DDLI);


        robot.frontrightDrive.setPower(0.25);
        robot.backrightDrive.setPower(-0.25);
        robot.frontleftDrive.setPower(0.25);
        robot.backleftDrive.setPower(-0.25);

        sleep(1500);
        //Start backwards
        //Goes backwards:

        drivePIDtime( .25, 0, -1, 2200);

        //Move 2+ tiles
        robot.frontrightDrive.setPower(0);
        robot.backrightDrive.setPower(0);
        //leftMotor.setPower(0);
        robot.frontleftDrive.setPower(0);
        robot.backleftDrive.setPower(0);
        //Grab the Build plate

        sleep(750);
        robot.dragDriveRight.setPosition(robot.DDRD);
        robot.dragDriveLeft.setPosition(robot.DDLD);

        sleep(2000);
        //Go forwards (backwards)
        drivePIDtime( .35, 0, 1, 2250);

        robot.frontrightDrive.setPower(0);
        robot.backrightDrive.setPower(0);
        //leftMotor.setPower(0);
        robot.frontleftDrive.setPower(0);
        robot.backleftDrive.setPower(0);

        //Release build plate
        robot.dragDriveRight.setPosition(robot.DDRI);
        robot.dragDriveLeft.setPosition(robot.DDLI);

        sleep(1000);

        //Strafe to under bridge:
        robot.frontrightDrive.setPower(-0.5);
        robot.backrightDrive.setPower(0.5);
        robot.frontleftDrive.setPower(-0.5);
        robot.backleftDrive.setPower(0.5);

        sleep(2500);//500 at 0.5 power gives a bit less than one tile, so 22.75" minus 2" or 3"


        // turn the motors off.
        //rightMotor.setPower(0);
        robot.frontrightDrive.setPower(0);
        robot.backrightDrive.setPower(0);
        //leftMotor.setPower(0);
        robot.frontleftDrive.setPower(0);
        robot.backleftDrive.setPower(0);
    }
}