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
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor.*;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.android.dx.io.instructions.OneRegisterDecodedInstruction;
import org.firstinspires.ftc.teamcode.MathFunctions.*;
import org.firstinspires.ftc.teamcode.OmniHardwarePushbot.*;
import org.firstinspires.ftc.teamcode.MiniPID.*;
import android.graphics.Color;


@Autonomous(name="Actually Competent Auto", group="PID")
//@Disabled
public class OmniAutonomousSimplePID_mike extends LinearOpMode {

    public double z_angle;
    public double globalAngle;
    OmniHardwarePushbot robot = new OmniHardwarePushbot();

    MiniPID controllerAngle = new MiniPID(0.035, 0, 0.03); //.025
    MiniPID controllerDrive = new MiniPID(0.035, 0, 0); //.025
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

    public double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle;

        return -MathFunctions.AngleWrap(deltaAngle - globalAngle);
    }

    public int getColor(int sensor){

        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 500;
        if(sensor == 1) {
            Color.RGBToHSV((int) (robot.colorSensor.red() * SCALE_FACTOR),
                    (int) (robot.colorSensor.green() * SCALE_FACTOR),
                    (int) (robot.colorSensor.blue() * SCALE_FACTOR),
                    hsvValues);
        }
        double hue = hsvValues[0];

        //hue is now the value we need to determine what color block we are looking at (light on)

        if((hue >= 35) && (hue <=55)){
            return 1; //Yellow skyblock
        }
        else if(hue > 135){
            return 2; //Black Skystone
        }
        else{
            return 0; //Nothing
        }



    }

    public double getDistanceColor(int sensor){
        if(sensor == 1){
            double distance = robot.distanceSensor_color.getDistance(DistanceUnit.CM);
            return distance;
        }
        else{
            return 0;
        }
    }
    public double getDistance(int sensor){
        if(sensor == 1){
            double distance = robot.distanceSensor.getDistance(DistanceUnit.CM);
            return distance;
        }
        else if(sensor ==2){
            double distance = robot.distanceSensor_2.getDistance(DistanceUnit.CM);
            return distance;
        }
        else{
            return 0;
        }

    }

    public void drivePID(double power, double goalAngle) {//-180 to 180
        double starTime = System.currentTimeMillis();
        controllerDrive.setOutputLimits(-1,1);
        while (true) {
            double correction = controllerDrive.getOutput(getAngle(), goalAngle);
            telemetry.addData("Angle:", getAngle()); //Gives our current pos
            telemetry.addData("Hot Garb:", correction);
            telemetry.addData("Global Subtract", globalAngle);
            telemetry.update();
            double y = power;
            double x = 0;
            double z = correction;
            robot.frontleftDrive.setPower(y + x + z);
            robot.frontrightDrive.setPower(-y + x + z);
            robot.backleftDrive.setPower(y - x + z);
            robot.backrightDrive.setPower(-y - x + z);
        }
    }
    public void strafePIDLeft() {//-180 to 180
        robot.frontrightDrive.setPower(-0.5);
        robot.backrightDrive.setPower(0.5);
        robot.frontleftDrive.setPower(-0.5);
        robot.backleftDrive.setPower(0.5);
    }
    public void strafePIDRight() {//-180 to 180
        robot.frontrightDrive.setPower(0.5);
        robot.backrightDrive.setPower(-0.5);
        robot.frontleftDrive.setPower(0.5);
        robot.backleftDrive.setPower(-0.5);
    }
    public void stopDrive(){
        robot.frontleftDrive.setPower(0);
        robot.frontrightDrive.setPower(0);
        robot.backleftDrive.setPower(0);
        robot.backrightDrive.setPower(0);
    }
    public void grabBlock(){
        robot.autoClaw.setPosition(robot.autoClawIdle);
        sleep(500);
        robot.autoClawArm.setPosition(robot.autoArmDown);
        sleep(2000);
        robot.autoClaw.setPosition(robot.autoClawGrab);
        sleep(1000);
        robot.autoClawArm.setPosition(robot.autoArmIdle);
        sleep(2000);
    }

    public void dropBlock(){
        robot.autoClawArm.setPosition(robot.autoArmDown);
        sleep(2000);
        robot.autoClaw.setPosition(robot.autoClawIdle);
        sleep(1000);
        robot.autoClawArm.setPosition(robot.autoArmIdle);
        sleep(2000);
        robot.autoClaw.setPosition(robot.autoClawGrab);
        sleep(2000);
    }
    public void turnToAnglePID(double goalAngle){//-180 to 180
        controllerAngle.setOutputLimits(-1,1);
        while (true) {

            double hotGarb = controllerAngle.getOutput(getAngle(), goalAngle);

            telemetry.addData("Angle:", getAngle()); //Gives our current pos
            telemetry.addData("Hot Garb:", hotGarb);
            telemetry.addData("Global Subtract", globalAngle);
            telemetry.update();


            hotGarb =hotGarb*robot.turnFactorPID ;

            robot.frontrightDrive.setPower(hotGarb);
            robot.backrightDrive.setPower(hotGarb);
            robot.frontleftDrive.setPower(hotGarb);
            robot.backleftDrive.setPower(hotGarb);

            if( ((goalAngle - robot.tolerancePID) <= getAngle()) && ((goalAngle + robot.tolerancePID) >= getAngle() )){
                break;
            }

            if(getAngle() == goalAngle){
                robot.frontrightDrive.setPower(0);
                robot.backrightDrive.setPower(0);
                robot.frontleftDrive.setPower(0);
                robot.backleftDrive.setPower(0);
                break;
            }

        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        setAngle();
        //Code above here should never change
        while(true) {
            strafePIDLeft();
            if(getDistanceColor(1) < 60){
                break;
            }
            telemetry.addData("Color", getDistanceColor(1));
            telemetry.update();

        }
    }
}