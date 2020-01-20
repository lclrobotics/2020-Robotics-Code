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

@Autonomous(name="Omni Auto Simple", group="Exercises")
//@Disabled
public class OmniAutonomousSimpleTape extends LinearOpMode
{
    public double z_angle;

    OmniHardwarePushbot robot = new OmniHardwarePushbot();


    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException
    {
        robot.init(hardwareMap);
        waitForStart();

        robot.autoClaw.setPosition(0.07);
        sleep(100);

        //turn(180);

        robot.frontrightDrive.setPower(-0.5);
        robot.backrightDrive.setPower(-0.5);
        robot.frontleftDrive.setPower(0.5);
        robot.backleftDrive.setPower(0.5);

        sleep(500);//500 at 0.5 power gives a bit less than one tile, so 22.75" minus 2" or 3"


        // turn the motors off.
        //rightMotor.setPower(0);
        robot.frontrightDrive.setPower(0);
        robot.backrightDrive.setPower(0);
        //leftMotor.setPower(0);
        robot.frontleftDrive.setPower(0);
        robot.backleftDrive.setPower(0);
    }
}