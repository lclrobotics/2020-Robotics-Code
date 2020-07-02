package org.firstinspires.ftc.teamcode.RobotUtilities;

public class StonePriority {

    public static int i;
    private static int stone;
    private static int priority;

    /**
     * analyzeStoneLayout () simply is used to check for the distance the robot is from the wall
     * to see what orientation the stones are set into
     * If robot fails to scan for skystones, the robot will simply prioritize the closest stones
     */
    public static void analyzeStoneLayout () {
        if ((MathFunctions.getDistance(2) >= 70) && (MathFunctions.getDistance(2) <= 90)){
            stone = 1;
        }
        if ((MathFunctions.getDistance(2) >= 90) && (MathFunctions.getDistance(2) <= 110)) {
            stone = 2;
        }
        if ((MathFunctions.getDistance(2) >= 110) && (MathFunctions.getDistance(2) <= 130)) {
            stone = 3;
        }
        else {
            stone = 4;
        }
    }

    /**
     * prioritize set the priority of the stones to grab the skystones first and then the closest
     * to the build site second
     * @return prioritized stone number
     */
    public static double prioritize (){

        if (stone == 1){
            if (i == 1) priority = 6;
            if (i == 2) priority = 3;
            if (i == 3) priority = 5;
            if (i == 4) priority = 4;
            if (i == 5) priority = 2;
            if (i == 6) priority = 1;
        }
        if (stone == 2) {
            if (i == 1) priority = 5;
            if (i == 2) priority = 2;
            if (i == 3) priority = 6;
            if (i == 4) priority = 4;
            if (i == 5) priority = 3;
            if (i == 6) priority = 1;
        }
        if (stone == 3){
            if (i == 1) priority = 4;
            if (i == 2) priority = 1;
            if (i == 3) priority = 6;
            if (i == 4) priority = 5;
            if (i == 5) priority = 3;
            if (i == 6) priority = 2;
        }
        if (stone == 4){
            if (i == 1) priority = 6;
            if (i == 2) priority = 5;
            if (i == 3) priority = 4;
            if (i == 4) priority = 3;
            if (i == 5) priority = 2;
            if (i == 6) priority = 1;
        }

        return priority;
    }

        public static int stoneDistance = 10+((priority-1)*20);

}