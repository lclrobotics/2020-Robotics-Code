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

@Autonomous(name="Foundation Red 15s", group="Foundation")
//@Disabled
public class OmniAutoFoundationRed_15s extends LinearOpMode
{
    public double z_angle;

    OmniHardwarePushbot robot = new OmniHardwarePushbot();


    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException
    {
        robot.init(hardwareMap);
        waitForStart();
        sleep(15000);

        robot.dragDriveRight.setPosition(robot.DDRI);
        robot.dragDriveLeft.setPosition(robot.DDLI);

        robot.frontrightDrive.setPower(-0.25);
        robot.backrightDrive.setPower(0.25);
        robot.frontleftDrive.setPower(-0.25);
        robot.backleftDrive.setPower(0.25);

        sleep(1500);

        robot.autoClaw.setPosition(0.07);
        sleep(100);
        //Start backwards
        //Goes backwards:

        robot.frontrightDrive.setPower(-0.5);
        robot.backrightDrive.setPower(-0.5);
        robot.frontleftDrive.setPower(0.5);
        robot.backleftDrive.setPower(0.5);

        sleep(1000);//500 at 0.5 power gives a bit less than one tile, so 22.75" minus 2" or 3"
        //Move 2+ tiles
        robot.frontrightDrive.setPower(0);
        robot.backrightDrive.setPower(0);
        //leftMotor.setPower(0);
        robot.frontleftDrive.setPower(0);
        robot.backleftDrive.setPower(0);
        //Grab the Build plate
        robot.dragDriveRight.setPosition(robot.DDRD);
        robot.dragDriveLeft.setPosition(robot.DDLD);

        sleep(2000);
        //Go forwards (backwards)
        robot.frontrightDrive.setPower(0.5);
        robot.backrightDrive.setPower(0.5);
        robot.frontleftDrive.setPower(-0.5);
        robot.backleftDrive.setPower(-0.5);

        sleep(2000);//500 at 0.5 power gives a bit less than one tile, so 22.75" minus 2" or 3"


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

        sleep(2200);//500 at 0.5 power gives a bit less than one tile, so 22.75" minus 2" or 3"


        // turn the motors off.
        //rightMotor.setPower(0);
        robot.frontrightDrive.setPower(0);
        robot.backrightDrive.setPower(0);
        //leftMotor.setPower(0);
        robot.frontleftDrive.setPower(0);
        robot.backleftDrive.setPower(0);
    }
}