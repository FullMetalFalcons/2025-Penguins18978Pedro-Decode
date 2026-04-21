package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Location Chooser", group = "OpModes")
public class LocationChooser extends OpMode {

    // Define starting position names
    public enum StartingLocation {
        BLUE_GOAL,
        BLUE_WALL,
        RED_GOAL,
        RED_WALL
    }

    // Define possible starting coordinate positions
    // Use PedroPathing's Pose class because it is easier to mirror across the field
    public static final Pose BLUE_GOAL_STARTING_POSE = new Pose(16, 113, Math.toRadians(0));
    public static final Pose BLUE_WALL_STARTING_POSE = new Pose(56, 9, 0);
    public static final Pose RED_GOAL_STARTING_POSE = BLUE_GOAL_STARTING_POSE.mirror();
    public static final Pose RED_WALL_STARTING_POSE = BLUE_WALL_STARTING_POSE.mirror();

    // Define goal positions for Z-Targeting System
    public static final Pose BLUE_GOAL_POSE = new Pose(5, 139);
    public static final Pose RED_GOAL_POSE = BLUE_GOAL_POSE.mirror();

    // Define launch positions for Autonomous and TeleOp alignment
    public static final Pose BLUE_LAUNCH_POSE = new Pose(48, 96, Math.toRadians(135));
    public static final Pose RED_LAUNCH_POSE = BLUE_LAUNCH_POSE.mirror();

    public static StartingLocation chosenStartingLocation = StartingLocation.BLUE_GOAL;
    public static Pose chosenStartingPose = BLUE_GOAL_STARTING_POSE;
    public static Pose chosenGoalPose = BLUE_GOAL_POSE;
    public static Pose chosenLaunchPose = BLUE_LAUNCH_POSE;
    public static double chosenDriverHeading = 180;
    private int locationNumber = 1;



    // Menu control variables
    double lastLeftStickY1;
    double lastLeftStickY2;
    boolean upWasInputted;
    boolean downWasInputted;
    final double STICK_THRESHOLD = 0.5;

    // Runs once when INIT is pressed
    @Override
    public void init() {

        //

    }

    // Runs continuously after INIT is pressed and before START is pressed
    @Override
    public void init_loop() {

        // Allow the menu to be navigated via the dpad or joystick
        updateInputs();
        if (downWasInputted) {  // Pressing down increases number because the order is   1 -> 2 -> 3 -> 4
            locationNumber ++;
            if (locationNumber > 4) locationNumber = 1;
        }
        if (upWasInputted) {    // Pressing up decreases number because the order is   1 -> 2 -> 3 -> 4
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
                chosenStartingPose = BLUE_GOAL_STARTING_POSE;
                chosenStartingLocation = StartingLocation.BLUE_GOAL;
                telemetry.addLine("Blue Goal successfully selected!");
                break;
            case 2:
                chosenStartingPose = BLUE_WALL_STARTING_POSE;
                chosenStartingLocation = StartingLocation.BLUE_WALL;
                telemetry.addLine("Blue Wall successfully selected!");
                break;
            case 3:
                chosenStartingPose = RED_GOAL_STARTING_POSE;
                chosenStartingLocation = StartingLocation.RED_GOAL;
                telemetry.addLine("Red Goal successfully selected!");
                break;
            case 4:
                chosenStartingPose = RED_WALL_STARTING_POSE;
                chosenStartingLocation = StartingLocation.RED_WALL;
                telemetry.addLine("Red Wall successfully selected!");
                break;
        }

        // Setup goal and driver location based on alliance color
        if (chosenStartingLocation == StartingLocation.BLUE_GOAL || chosenStartingLocation == StartingLocation.BLUE_WALL) {
            chosenGoalPose = BLUE_GOAL_POSE;
            chosenLaunchPose = BLUE_LAUNCH_POSE;
            chosenDriverHeading = 180;
        } else {
            chosenGoalPose = RED_GOAL_POSE;
            chosenLaunchPose = RED_LAUNCH_POSE;
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

    private void updateInputs() {
        // Record input edge detection for both gamepads' joysticks
        boolean joystickWasTiltedUp = (-gamepad1.left_stick_y > STICK_THRESHOLD && lastLeftStickY1 < STICK_THRESHOLD) ||
                                      (-gamepad2.left_stick_y > STICK_THRESHOLD && lastLeftStickY2 < STICK_THRESHOLD);

        boolean joystickWasTiltedDown = (-gamepad1.left_stick_y < -STICK_THRESHOLD && lastLeftStickY1 > -STICK_THRESHOLD) ||
                                      (-gamepad2.left_stick_y < -STICK_THRESHOLD && lastLeftStickY2 > -STICK_THRESHOLD);

        // Factor in all methods of input (both joysticks, both D-pads)
        upWasInputted =   joystickWasTiltedUp   || gamepad1.dpadUpWasPressed()   || gamepad2.dpadUpWasPressed();
        downWasInputted = joystickWasTiltedDown || gamepad1.dpadDownWasPressed() || gamepad2.dpadDownWasPressed();

        lastLeftStickY1 = -gamepad1.left_stick_y;
        lastLeftStickY2 = -gamepad2.left_stick_y;
    }

} // end class