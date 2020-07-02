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

package org.firstinspires.ftc.teamcode.Other;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoverRuckus.OmniHardwarePushbot;

@Autonomous(name="Omni Drive Autonomous", group="Exercises")
@Disabled
public class OmniAutonomous extends LinearOpMode
{
    public double z_angle;

    OmniHardwarePushbot robot = new OmniHardwarePushbot();


    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException
    {
        robot.init(hardwareMap);
        waitForStart();

        robot.autoClaw.setPosition(0.07);//closes claw so it can't hit the skybridge
        sleep(100);

        //turn(180);

        robot.frontrightDrive.setPower(0.5);
        robot.backrightDrive.setPower(0.5);
        robot.frontleftDrive.setPower(-0.5);
        robot.backleftDrive.setPower(-0.5);

        sleep(500);//500 at 0.5 power a bit less than one tile, so 22.75" minus 2" or 3", will use 20"


        // turn the motors off.
        //rightMotor.setPower(0);
        robot.frontrightDrive.setPower(0);
        robot.backrightDrive.setPower(0);
        //leftMotor.setPower(0);
        robot.frontleftDrive.setPower(0);
        robot.backleftDrive.setPower(0);
    }

    //
//public void turn(int degrees) {
//double z_angle = 0;
//while ((int)(z_angle) != degrees){
//z_angle = (robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle +
//robot.imu2.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) / 2;
//
//if(z_angle < degrees) {
//robot.frontrightDrive.setPower(-0.5);
//robot.backrightDrive.setPower(-0.5);
//robot.frontleftDrive.setPower(0.5);
//robot.backleftDrive.setPower(0.5);
//} else {
//robot.frontrightDrive.setPower(0.5);
//robot.backrightDrive.setPower(0.5);
//robot.frontleftDrive.setPower(-0.5);
//robot.backleftDrive.setPower(-0.5);
//}
//}
//robot.frontrightDrive.setPower(0);
//robot.backrightDrive.setPower(0);
//robot.frontleftDrive.setPower(0);
//robot.backleftDrive.setPower(0);
//
//}
//
}