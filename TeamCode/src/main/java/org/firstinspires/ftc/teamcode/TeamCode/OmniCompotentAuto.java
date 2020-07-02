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

package org.firstinspires.ftc.teamcode.TeamCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoverRuckus.OmniHardwarePushbot;
import org.firstinspires.ftc.teamcode.RobotUtilities.MathFunctions;
import org.firstinspires.ftc.teamcode.RobotUtilities.RobotMovement;
import org.firstinspires.ftc.teamcode.RobotUtilities.StonePriority;

@Autonomous(name="Actually Competent Auto", group="PID")
//@Disabled
public class OmniCompotentAuto extends LinearOpMode {

    OmniHardwarePushbot robot = new OmniHardwarePushbot();

    @Override
    public void runOpMode() {

        //Initialize servos to starting positions
        robot.init(hardwareMap);
        robot.autoClawArm.setPosition(robot.autoArmIdle);
        robot.autoClaw.setPosition(robot.autoClawIdle);
        robot.dragDriveLeft.setPosition(robot.DDRI);
        robot.dragDriveLeft.setPosition(robot.DDLI);
        waitForStart();
        MathFunctions.setAngle();

        //Code above here should never change

        //While the stop button isn't pressed, run this code
            while (!isStopRequested()) {


                //Set i to 1 and strafe to the stones
                StonePriority.i = 1;
                robot.autoClawArm.setPosition(robot.autoArmDown);
                RobotMovement.strafePID(.5, 0, -1, 40);

                //Drive along stones until we see a skystone
                while ((MathFunctions.getColor(1) != 2) || (MathFunctions.getDistance(2) <= 30)){
                    RobotMovement.drivePIDGeneral(.25, 0, 1);
                }

                //Check for the position of the skystones
                StonePriority.analyzeStoneLayout();

                //Set the priority of the stones for the robot to grab
                StonePriority.prioritize();

                //Start loop
                while (StonePriority.i<=4){             //Set i to desired stone count

                    //Set i to the next iteration
                    StonePriority.i = StonePriority.i+1;
                    //Pick up stone
                    robot.autoClaw.setPosition(robot.autoClawGrab);
                    sleep(500);
                    robot.autoClawArm.setPosition(robot.autoArmIdle);

                    //Get away from stones and move towards build site
                    RobotMovement.strafeRight(1, 300);
                    RobotMovement.drivePIDtime(1, 0, -1, 1000, 1);
                    RobotMovement.drivePID(.5, 0, -1, 125-(StonePriority.i*20), 1);
                    sleep(500);

                    //Move towards foundation and drop stone
                    RobotMovement.strafePID(.4, 0, -1, 50);
                    robot.autoClawArm.setPosition(robot.autoArmDeliver);
                    robot.autoClaw.setPosition(robot.autoClawIdle);

                    //reset the priority of the stones for the robot to grab
                    StonePriority.prioritize();

                    //Move away from foundation and back to Stones
                    RobotMovement.strafeRight(1, 300);
                    RobotMovement.drivePIDtime(1, 0, 1, 1000, 1);
                    RobotMovement.drivePID(.5, 0, 1, StonePriority.stoneDistance, 2);
                    robot.autoClawArm.setPosition(robot.autoArmDown);

                    //Strafe to stones and go back to top of loop
                    RobotMovement.strafePID(.5, 0, -1, 35);
                    //StonePriority.i = StonePriority.i+1;

                }

                //Pick up stone
                robot.autoClaw.setPosition(robot.autoClawGrab);
                sleep(500);
                robot.autoClawArm.setPosition(robot.autoArmIdle);

                //Get away from stones and move towards build site
                RobotMovement.strafeRight(1, 300);
                RobotMovement.drivePIDtime(1, 0, -1, 1000, 1);
                RobotMovement.drivePID(.5, 0, -1, 125-(StonePriority.i*20), 1);
                sleep(500);

                //Move towards foundation and drop stone
                RobotMovement.strafePID(.4, 0, -1, 50);
                robot.autoClawArm.setPosition(robot.autoArmDeliver);
                robot.autoClaw.setPosition(robot.autoClawIdle);

                //Move away foundation and prepare to grab foundation
                RobotMovement.strafeRight(1, 350);
                RobotMovement.turnToAnglePID(90);

                //Drive to and grab foundation
                RobotMovement.drivePIDtime(.25, 90, -1, 1500, 1);
                robot.dragDriveLeft.setPosition(robot.DDRD);
                sleep(200);
                robot.dragDriveLeft.setPosition(robot.DDLD);
                sleep(200);

                //Move foundation to build zone
                RobotMovement.drivePIDtime( .35, 90, 1, 2250, 1);
                RobotMovement.stopDrive();

                //Release foundation
                robot.dragDriveRight.setPosition(robot.DDRI);
                sleep(200);
                robot.dragDriveLeft.setPosition(robot.DDLI);
                sleep(200);

                //Park
                RobotMovement.drivePIDtime(.5, 90, -1, 250, 0);
                RobotMovement.strafeLeft(1, 1250);

                // Dont put code below here
                RobotMovement.stopDrive();
                break;
            }
            RobotMovement.stopDrive();
            stop();
    }
}