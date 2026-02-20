package org.firstinspires.ftc.teamcode;



public class LinearCalc {
    private double robotX;
    private double robotY;
    public int goalX;
    public int goalY = 138;

    public LinearCalc(double robotX, double robotY, boolean blue) {
        this.robotX = robotX;
        this.robotY = robotY;
        if (blue) {
            int goalX = 12;
        }
        if (!blue) {
            int goalX = 131;
        }
    }

    public int calc() {

        double a = Math.abs(robotX - goalX);
        double b = Math.abs(robotY - goalY);
        double c = Math.sqrt(a*a + b*b);

        double miku = 8*c + 2260;

        if (miku <= 0) {
            miku = 2260;
        }

        return Math.toIntExact(Math.round(miku));
    }
}