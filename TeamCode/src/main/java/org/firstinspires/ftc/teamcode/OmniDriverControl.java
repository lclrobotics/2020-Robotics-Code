package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


/**
 * This is the Main Driver Control for the 2020 FTC Skystone Robot Riccochet
 */

@TeleOp(name="Omni_Driver_Control", group="Iterative Opmode")//Originally just DriverControl
//@Disabled
public class OmniDriverControl extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();


    //Creates new robot
    OmniHardwarePushbot robot       = new OmniHardwarePushbot();//probably want to update class + function names

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        //Initalize hardware from Hardware Pushbot
        robot.init(hardwareMap);

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
        robot.autoClawArm.setPosition(robot.autoClawIdle); //Set Idle Position
        robot.autoClaw.setPosition(robot.autoClawIdle); //Set Idle Position
        robot.delivRack.setPower(0.5);
        robot.delivClaw.setPosition(robot.delivIdle);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    public boolean changed1 = false;
    public boolean changed2 = false;
    public boolean changed3 = false;
    public boolean changed4 = false;


    @Override
    public void loop() {

        //Gets z Values - Right-Handed Coordinate System
        Orientation imu_angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        Orientation imu2_angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double z_angle = (imu_angles.firstAngle + imu2_angles.firstAngle) / 2;//zAngle;

        //Double Variables for driver control sticks
        double x  =  gamepad1.left_stick_x;
        double y  =  gamepad1.left_stick_y;
        double z  =  gamepad1.right_stick_x;

        double yLift = gamepad2.left_stick_y;

        double yDeliv = gamepad2.right_stick_y;
        /*
        NEWLY COMMENTED 1/10/20
        //Math for robot orientated drive. The Z axis offset is converted to radians. Then by multiplying the y and x values by the
        //cos and sin of the gyro, we can "rotate" the gamepad left stick, so forward on the sick is always away
        double rad = Math.toRadians(z_angle)
        double forward = (y*Math.cos(rad))+(x*Math.sin(rad));
        double side    = (-y*Math.sin(rad))+(x*Math.cos(rad));


        //Assigning drive power to motors
        robot.frontleftDrive.setPower(-forward-side-z);
        robot.frontrightDrive.setPower(-forward+side-z);
        robot.backleftDrive.setPower(forward-side-z);
        robot.backrightDrive.setPower(forward+side-z);

        */

        robot.delivRack.setPower(yDeliv);

        if (gamepad1.left_bumper) { // Control Speed of Drive
            robot.speedFactor = 3;
        }
        else{
            robot.speedFactor = 1;
        }


        robot.frontleftDrive.setPower((Math.pow(y+x+z, 3))/robot.speedFactor);
        robot.frontrightDrive.setPower((Math.pow(-y+x+z,3))/robot.speedFactor);
        robot.backleftDrive.setPower((Math.pow(y-x+z, 3))/robot.speedFactor);
        robot.backrightDrive.setPower((Math.pow(-y-x+z, 3))/robot.speedFactor);

        robot.lift.setPower(-yLift);

        //All the if statements for the different servos
        if(gamepad2.x && !changed1) {
            robot.autoClaw.setPosition(robot.autoClawIdle != robot.autoClaw.getPosition() ? robot.autoClawIdle : robot.autoClawGrab);
            changed1 = true;
        } else if(!gamepad2.x) changed1 = false;

        if(gamepad2.a && !changed2) {
            robot.autoClawArm.setPosition(robot.autoArmIdle != robot.autoClawArm.getPosition() ? robot.autoArmIdle : robot.autoArmDown);
            changed2 = true;
        } else if(!gamepad2.a) changed2 = false;

        if(gamepad2.y && !changed3) {
            robot.dragDriveRight.setPosition(robot.DDRI != robot.dragDriveRight.getPosition() ? robot.DDRI : robot.DDRD);
            robot.dragDriveLeft.setPosition( robot.DDLI != robot.dragDriveLeft.getPosition() ? robot.DDLI : robot.DDLD);
            changed3 = true;
        } else if(!gamepad2.y) changed3 = false;

        if(gamepad2.b && !changed4) {
            robot.delivClaw.setPosition(robot.delivIdle != robot.delivClaw.getPosition() ? robot.delivIdle : robot.delivGrab);
            changed4 = true;
        } else if(!gamepad2.b) changed4 = false;

        if (gamepad2.right_bumper) {
            robot.rightIntake.setPower(0.45);
            robot.leftIntake.setPower(0.45);
        } else if (gamepad2.left_bumper){
            robot.rightIntake.setPower(-.35);
            robot.leftIntake.setPower(-.35);
        } else {
            robot.rightIntake.setPower(0);
            robot.leftIntake.setPower(0);
        }

        //telemetry.addData("Front Left", robot.frontleftDrive.getCurrentPosition());
        //telemetry.addData("Front Right", robot.frontrightDrive.getCurrentPosition());
        //telemetry.addData("Back Left", robot.backleftDrive.getCurrentPosition());
        //telemetry.addData("Back Right", robot.backrightDrive.getCurrentPosition());
        telemetry.addData("Auto Claw" , robot.autoClawIdle != robot.autoClaw.getPosition());
        telemetry.addData("Auto Claw Arm" , robot.autoArmIdle != robot.autoClawArm.getPosition());
        telemetry.addData("Base Plate" , robot.DDRI != robot.dragDriveRight.getPosition());
        telemetry.addData("Delivery" , robot.delivIdle != robot.delivClaw.getPosition());
        telemetry.addData("Orientation", z_angle);
        telemetry.update();
    }
    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}