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

@Autonomous(name="Omni Auto Full Red", group="Full")
//@Disabled
public class OmniAutonomousFullRed extends LinearOpMode
{
    public double z_angle;

    OmniHardwarePushbot robot = new OmniHardwarePushbot();
    public void drive(double power, int angle, int time){ // Time is in seconds; Direction is non field oriented
        double rad = Math.toRadians(angle);
        double y = -Math.cos(rad);
        double x = Math.sin(rad);

        robot.frontrightDrive.setPower((-y+x)*power);
        robot.backrightDrive.setPower((-y-x)*power);
        robot.frontleftDrive.setPower((y+x)*power);
        robot.backleftDrive.setPower((y-x)*power);

        sleep(time);

        robot.frontrightDrive.setPower(0);
        robot.backrightDrive.setPower(0);
        robot.frontleftDrive.setPower(0);
        robot.backleftDrive.setPower(0);

        sleep(250);



    }
    public void stopDrive(){ // Time is in seconds; Direction is non field oriented
        robot.frontrightDrive.setPower(0);
        robot.backrightDrive.setPower(0);
        robot.frontleftDrive.setPower(0);
        robot.backleftDrive.setPower(0);
    }


    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException
    {
        robot.init(hardwareMap);
        waitForStart();
        sleep(100);

        drive(.5, 0, 1000);
        drive(.5, 90, 1000);
        drive(.5, 180, 1000);
        drive(.5, 270, 1000);
        drive(.5, 45, 1000);
        drive(.5, 225, 1000);
        stop();

    }
}