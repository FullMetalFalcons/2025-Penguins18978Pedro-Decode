package org.firstinspires.ftc.teamcode;



public class LinearCalc {

    // Constants based on the distances and velocities used in Autonomous
    final double CLOSE_LAUNCH_DISTANCE = 60.0;
    final int CLOSE_LAUNCH_VELOCITY = 2700;
    final double FAR_LAUNCH_DISTANCE = 130.0;
    final int FAR_LAUNCH_VELOCITY = 3300;
    final double DISTANCE_DIFFERENCE = FAR_LAUNCH_DISTANCE - CLOSE_LAUNCH_DISTANCE;
    final int VELOCITY_DIFFERENCE = FAR_LAUNCH_VELOCITY - CLOSE_LAUNCH_VELOCITY;

    private double robotX;
    private double robotY;
    public double goalX;
    public double goalY = LocationChooser.BLUE_GOAL_POSE.getY();


    // Constructor
    public LinearCalc(double robotX, double robotY, boolean blue) {
        this.robotX = robotX;
        this.robotY = robotY;
        if (blue) {
            goalX = LocationChooser.BLUE_GOAL_POSE.getX();
        } else {
            goalX = LocationChooser.RED_GOAL_POSE.getX();
        }
    }

    // Change the robot's position. Only needed if an instance of this class is planning on being reused
    public void setRobotPosition(double x, double y) {
        robotX = x;
        robotY = y;
    }

    /** Calculates the velocity in rpm that the flywheels should be run at based on the robot's distance from the goal
     * @param x the robot's x position according to the Pedro coordinate system
     * @param y the robot's y position according to the Pedro coordinate system */
    public int calc(double x, double y) {

        // Calculate the robot's distance to the goal using the Pythagorean Theorem
        // Absolute value is not needed because we square the values afterwards
        double a = goalX - x;
        double b = goalY - y;
        double distance = Math.sqrt(a*a + b*b);

        // Get a percentage of how far the robot is from the close launch distance
        // 0.0 is at the close distance, 1.0 is at the far distance
        double multiplier = (distance - CLOSE_LAUNCH_DISTANCE) / DISTANCE_DIFFERENCE;
        multiplier = Math.min(0, multiplier);

        // Calculate the needed velocity
        double power = CLOSE_LAUNCH_VELOCITY + (VELOCITY_DIFFERENCE * multiplier);
        return Math.toIntExact(Math.round(power));
    }

    /** Calculates the velocity in rpm that the flywheels should be run at based on the robot's distance from the goal */
    public int calc() {
        return calc(robotX, robotY);
    }


    public double clamp(double min, double value, double max) {
        return Math.max( Math.min(value, max), min );
    }
}