package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "Location Chooser", group = "OpModes")
public class AALocationChooser extends OpMode {

    // Define starting position names
    public enum StartingLocation {
        BLUE_GOAL,
        BLUE_WALL,
        RED_GOAL,
        RED_WALL
    }

    // Define possible starting coordinate positions
    private static final double PEDRO_CENTER = 72;
    public static final Pose2D BLUE_GOAL_STARTING = new Pose2D(DistanceUnit.INCH, PEDRO_CENTER-51, 122,
                                                      AngleUnit.DEGREES, 270+54);

    public static final Pose2D BLUE_WALL_STARTING = new Pose2D(DistanceUnit.INCH, PEDRO_CENTER-23, 8,
                                                      AngleUnit.DEGREES, 90);

    public static final Pose2D RED_GOAL_STARTING = new Pose2D(DistanceUnit.INCH, PEDRO_CENTER+51, 123,
                                                     AngleUnit.DEGREES, 270-54);

    public static final Pose2D RED_WALL_STARTING = new Pose2D(DistanceUnit.INCH, PEDRO_CENTER+23, 8,
                                                     AngleUnit.DEGREES, 90);

    // Define goal positions for Z-Targeting System
    public static final Pose2D BLUE_GOAL_POS = new Pose2D(DistanceUnit.INCH, PEDRO_CENTER-62, 137,
                                                          AngleUnit.DEGREES, 0);
    public static final Pose2D RED_GOAL_POS = new Pose2D(DistanceUnit.INCH, PEDRO_CENTER+62, 137,
                                                         AngleUnit.DEGREES, 0);

    public static StartingLocation chosenStartingLocation = StartingLocation.BLUE_GOAL;
    public static Pose2D chosenStartingPos = BLUE_GOAL_STARTING;
    public static Pose2D chosenGoalPos = BLUE_GOAL_POS;
    public static double chosenDriverHeading = 180;
    private int locationNumber = 1;

    // Runs once when INIT is pressed
    @Override
    public void init() {

        //

    }

    // Runs continuously after INIT is pressed and before START is pressed
    @Override
    public void init_loop() {

        // Allow the menu to be navigated via the dpad
        if (gamepad1.dpadDownWasPressed()) {
            locationNumber ++;
            if (locationNumber > 4) locationNumber = 1;
        }
        if (gamepad1.dpadUpWasPressed()) {
            locationNumber --;
            if (locationNumber < 1) locationNumber = 4;
        }

        // Display the menu options
        telemetry.addLine(menuLine("Blue Goal", 1));
        telemetry.addLine(menuLine("Blue Wall", 2));
        telemetry.addLine(menuLine("Red Goal", 3));
        telemetry.addLine(menuLine("Red Wall", 4));

        telemetry.update();
    }

    // Runs once when START is pressed
    @Override
    public void start() {

        // Set the robot and driver starting positions based on the menu selection
        switch (locationNumber) {
            case 1:
                chosenStartingPos = BLUE_GOAL_STARTING;
                chosenStartingLocation = StartingLocation.BLUE_GOAL;
                telemetry.addLine("Blue Goal successfully selected!");
                break;
            case 2:
                chosenStartingPos = BLUE_WALL_STARTING;
                chosenStartingLocation = StartingLocation.BLUE_WALL;
                telemetry.addLine("Blue Wall successfully selected!");
                break;
            case 3:
                chosenStartingPos = RED_GOAL_STARTING;
                chosenStartingLocation = StartingLocation.RED_GOAL;
                telemetry.addLine("Red Goal successfully selected!");
                break;
            case 4:
                chosenStartingPos = RED_WALL_STARTING;
                chosenStartingLocation = StartingLocation.RED_WALL;
                telemetry.addLine("Red Wall successfully selected!");
                break;
        }

        // Setup goal and driver location based on alliance color
        if (chosenStartingLocation == StartingLocation.BLUE_GOAL || chosenStartingLocation == StartingLocation.BLUE_WALL) {
            chosenGoalPos = BLUE_GOAL_POS;
            chosenDriverHeading = 180;
        } else {
            chosenGoalPos = RED_GOAL_POS;
            chosenDriverHeading = 0;
        }

        telemetry.update();

    }

    // Runs continually after START is pressed and before STOP is pressed
    @Override
    public void loop() {

        //

    }

    private String menuLine(String text, int number) {
        String selectorArrow = (number == locationNumber) ? "> " : "";
        return selectorArrow + text;
    }

} // end class