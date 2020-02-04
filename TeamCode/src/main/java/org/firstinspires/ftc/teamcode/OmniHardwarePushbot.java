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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class OmniHardwarePushbot
{
    /* Public OpMode members. */
    public BNO055IMU       imu;

    public ColorSensor colorSensor;
    public DistanceSensor distanceSensor_color;
    public DistanceSensor distanceSensor;
    public DistanceSensor distanceSensor_2;
    //public BNO055IMU       imu2;

    public DcMotor  frontleftDrive      = null;
    public DcMotor  frontrightDrive     = null;
    public DcMotor  backrightDrive      = null;
    public DcMotor  backleftDrive       = null;

    public DcMotor  rightIntake         = null;
    public DcMotor  leftIntake         = null;

    public DcMotor  lift                = null;

    public Servo    autoClaw            = null;
    public Servo    autoClawArm         = null ;

    public int      clawToggle          = 0;
    public int      clawToggleArm       = 0;
    public int      dragToggle          = 0;
    public int      speedFactor         = 1;

    public Servo    dragDriveRight      = null;
    public Servo    dragDriveLeft       = null;

    public CRServo    delivRack           = null;
    public Servo    delivClaw           = null;
    public AnalogInput pixyCam          = null;

    public double      autoClawIdle        = .5;
    public double      autoClawGrab        = .89;

    public double      autoArmIdle         = .39;
    public double      autoArmDown         = .82;//.83
    public double      autoArmDeliver      = .60;

    public double      DDRD                = .65;
    public double      DDLD                = .55;
    public double      DDRI                =   1;
    public double      DDLI                = .9;

    public double     delivIdle            = .6;
    public double     delivGrab            = .91;

    public double     turnFactorPID        = .5;

    public double     tolerancePID         = 2;
    public double     tolerancePID_d       = 1;

    /*
       We use cad on the team in many different ways. One of those ways is through
       creating custom parts like our auto claw, to maximize the ability of our robot.
       We also use cad to design our schools theatre set and make awards for band.
       With these things we are able to help the extracurriculars. +


       Not only have we been able to implement cad into our orbot design for ustom parts.
       but weve also been able to implement cad into our fundrasing, outreaching into our
       theatre program and outreaching into our band program to host a fun awards night.
    */

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public OmniHardwarePushbot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;



        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu = hwMap.get(BNO055IMU.class, "imu");
        //imu2 = hwMap.get(BNO055IMU.class, "imu2");
        imu.initialize(parameters);


        //imu2.initialize(parameters);
/*
        while(!imu.isGyroCalibrated() && !imu2.isGyroCalibrated()){
            //I don't know how necessary this is, decided to include it
        }
*/
        // Define and Initialize Motors
        colorSensor = hwMap.get(ColorSensor.class, "color_sensor");
        distanceSensor_color = hwMap.get(DistanceSensor.class, "color_sensor");
        distanceSensor = hwMap.get(DistanceSensor.class, "distance_sensor");
        distanceSensor_2 = hwMap.get(DistanceSensor.class, "distance_sensor_front");
        frontleftDrive        = hwMap.get(DcMotor.class, "front_left_drive");
        frontrightDrive       = hwMap.get(DcMotor.class, "front_right_drive");
        backleftDrive         = hwMap.get(DcMotor.class, "back_left_drive");
        backrightDrive        = hwMap.get(DcMotor.class, "back_right_drive");

        lift                  = hwMap.get(DcMotor.class, "lift");
        pixyCam               = hwMap.get(AnalogInput.class, "pixy");

        rightIntake           = hwMap.get(DcMotor.class, "right_intake");
        leftIntake            = hwMap.get(DcMotor.class, "left_intake");

        autoClaw              = hwMap.get(Servo.class, "auto_claw");
        autoClawArm           = hwMap.get(Servo.class, "auto_claw_arm");

        dragDriveRight           = hwMap.get(Servo.class, "right_drag");
        dragDriveLeft            = hwMap.get(Servo.class, "left_drag");

        delivRack             = hwMap.get(CRServo.class, "deliv_rack");
        delivClaw             = hwMap.get(Servo.class, "deliv_claw");

        frontleftDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        frontrightDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        backleftDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        backrightDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors

        lift.setDirection(DcMotor.Direction.FORWARD);

        rightIntake.setDirection(DcMotor.Direction.FORWARD);
        leftIntake.setDirection(DcMotor.Direction.REVERSE);
        // Set all motors to zero power
        frontleftDrive.setPower(0);
        frontrightDrive.setPower(0);
        backleftDrive.setPower(0);
        backrightDrive.setPower(0);

        lift.setPower(0);

        rightIntake.setPower(0);
        leftIntake.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        frontleftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontrightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backleftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backrightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.

    }
}
