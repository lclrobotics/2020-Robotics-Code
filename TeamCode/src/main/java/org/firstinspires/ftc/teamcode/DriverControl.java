/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

//@Disabled
@TeleOp(name="Driver_Control", group="Iterative Opmode")
@Disabled
public class DriverControl extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();


    //Creates new robot
    Hardware robot       = new Hardware();


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        //Begin Initilization
        telemetry.addData("Status", "Initializing");


        //Initalize hardware from Hardware Pushbot
        robot.init(hardwareMap);

        /*
        while(!robot.imu.isGyroCalibrated()) {// originally !isStopRequested() &&
            //originally had a sleep() and idle(), should be fine w/o it
        }
        */


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        //Double Variables for driver control sticks
        double x  =  gamepad1.left_stick_x;
        double y  =  gamepad1.left_stick_y;
        double z  =  gamepad1.right_stick_x;
        double intakeLeftTrig = gamepad2.left_trigger;
        double intakeRightTrig = gamepad2.right_trigger;

        //Arm and Out power doubles
        double rotate = gamepad2.left_stick_y;
        double extend = -gamepad2.right_stick_y;


        //Math for robot orientated drive. The Z axis offset is converted to radians. Then by multiplying the y and x values by the
        //cos and sin of the gyro, we can "rotate" the gamepad left stick, so forward on the sick is always away

        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double angel = angles.firstAngle;
        double pi = Math.PI;
        double rad = angel * (pi/180);
        double forward = (y*Math.cos(rad))+(x*Math.sin(rad));
        double side    = (-y*Math.sin(rad))+(x*Math.cos(rad));

        //Assigning drive power to motors
        robot.frontleftDrive.setPower(forward+side);
        robot.frontrightDrive.setPower(forward+side);
        robot.backleftDrive.setPower(forward-side);
        robot.backrightDrive.setPower(forward-side);

        //w/o the gyro:
        //robot.frontleftDrive.setPower(y+x-z);
        //robot.frontrightDrive.setPower(y-x+z);
        //robot.backleftDrive.setPower(y-x-z);
        //robot.backrightDrive.setPower(y+x+z);


        //Servo Position of Drag Drive
        if (gamepad2.y == true ) {
            telemetry.addData("Servos", "ON");
            robot.dragDrive.setPosition(.4);
            robot.dragDrive2.setPosition(.8);
        }
        else { //Go back up
            telemetry.addData("Servos", "OFF");
            robot.dragDrive.setPosition(.1);
            robot.dragDrive2.setPosition(.1);
        }

        //Intake
    }

    @Override
    public void stop() {
    }

}