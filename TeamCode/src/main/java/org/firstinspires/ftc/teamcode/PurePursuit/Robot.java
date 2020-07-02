package org.firstinspires.ftc.teamcode.PurePursuit;

public class Robot {
    public static boolean usingComputer = true;

    /**
     * Creates a robot simulation
     */
    public Robot(){
        worldXPosition = 100;
        worldYPosition = 140;
        worldAngle_rad = Math.toRadians(-180);
    }

    //The actual speed the robot is moving
    private double xSpeed = 0;
    private double ySpeed = 0;
    private double turnSpeed = 0;

    public static double worldXPosition;
    public static double worldYPosition;
    public static double worldAngle_rad;

}