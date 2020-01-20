package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
@Autonomous(name = "Test_Auto", group = "Test")
public class TestAuto extends OpMode {
    ColorSensor sensorColor;
    Hardware robot = new Hardware();

    public void encoder_drive (DcMotor motor, int dist, double power){ //distance is in mm
        int encoder_count = (int)(1120 * dist/ (100*Math.PI));
        motor.setTargetPosition(encoder_count);
        motor.setPower(power);
    }

    @Override
    public void init() {

        //New Robot
        robot.init(hardwareMap);
        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");
        robot.frontleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


    }

    @Override
    public void loop() {
        //int drive_dist = 304*6; //1 ft = 304.8 mm
        //double power = 0.5; //half power, I think
        //encoder_drive(robot.frontleftDrive, drive_dist, power);
        //encoder_drive(robot.frontrightDrive, drive_dist, power);
        //encoder_drive(robot.backleftDrive, drive_dist, power);
        //encoder_drive(robot.backrightDrive, drive_dist, power);
        telemetry.addData("Blue Level", sensorColor.blue()*255);
    }
}