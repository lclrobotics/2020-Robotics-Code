package org.firstinspires.ftc.teamcode.Skystone;

import com.qualcomm.hardware.bosch.BNO055IMU;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
@Disabled
public class Hardware
{
    /* Public OpMode members. */
    public BNO055IMU    imu;
    public DcMotor  frontleftDrive      = null;
    public DcMotor  frontrightDrive     = null;
    public DcMotor  backrightDrive      = null;
    public DcMotor  backleftDrive       = null;
    public Servo    dragDrive               = null;
    public Servo    dragDrive2 = null;
    public DcMotor  leftintake           = null;
    public DcMotor  rightintake     = null;
    public ColorSensor tapeDetect       = null;
    //Mid Position for Servo
    public static final double IDLE_SERVO       =  0.1 ;

    //Vuforia stuff
    public static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    public static final String LABEL_FIRST_ELEMENT = "Stone";
    public static final String LABEL_SECOND_ELEMENT = "Skystone";
    public static final String VUFORIA_KEY = "AYY933f/////AAABmTOVXdOJfkPVkUtta9P0Ov6A0Kkpraw6g8qaBdqKpss4R7ACOLNLYQFf8tC0kTybgRQMLYdeHo8S+6F/eWz4Sncn0yfO6zmozRW6USmQ1ur2y3rduGQhPmNR0/khvxtVr1Q79H5VqUxTDckX0X6aMryRfpv/1PRuobW8Uu5tH4yK0GKSUHOzN9pVp/oExdfk5wiC3JYFwXKwL6MoXVa10mBJhWd9Eu61EGtGmkZnMvZCO/wobi2TytCOCzyueDOCZ/YKdLtWZEPZfsCqaYGgVZ1/KmCEpzZk6+DVOI5aBhSrqEVPvXNDyaSJgPbs1TNKB3NkrbvDyYPcfKzrXqmj2bLjATjaMpR4i/UR37uHEwTg";
    public VuforiaLocalizer vuforia;
    public TFObjectDetector tfod;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public Hardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        //Define and Initialize Sensors
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode             = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit        = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit        = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled   = false;

        imu = hwMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        while(!imu.isGyroCalibrated()) {// originally !isStopRequested() &&
            //originally had a sleep() and idle(), should be fine w/o it
        }


        // Define and Initialize Motors
        frontleftDrive        = hwMap.get(DcMotor.class, "front_left_drive");
        frontrightDrive       = hwMap.get(DcMotor.class, "front_right_drive");
        backleftDrive         = hwMap.get(DcMotor.class, "back_left_drive");
        backrightDrive        = hwMap.get(DcMotor.class, "back_right_drive");
        dragDrive             = hwMap.get(Servo.class, "drag_drive");
        dragDrive2            = hwMap.get(Servo.class, "drag_drive_2");
        //leftintake       = hwMap.get(DcMotor.class, "left_intake");
        //rightintake      = hwMap.get(DcMotor.class, "right_intake");
        //tapeDetect            = hwMap.get(ColorSensor.class, "sensor_color_distance");
        //Direction
        frontleftDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        frontrightDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        backleftDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        backrightDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        //leftintake.setDirection(DcMotor.Direction.FORWARD);
        //rightintake.setDirection(DcMotor.Direction.FORWARD);
        // Set all motors to zero power
        frontleftDrive.setPower(0);
        frontrightDrive.setPower(0);
        backleftDrive.setPower(0);
        backrightDrive.setPower(0);
        //leftintake.setPower(0);
        //rightintake.setPower(0);
        dragDrive.setPosition(IDLE_SERVO);
        dragDrive2.setPosition(IDLE_SERVO);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        frontleftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontrightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backleftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backrightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //leftintake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //rightintake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Hardware robot       = new Hardware();

    }
}