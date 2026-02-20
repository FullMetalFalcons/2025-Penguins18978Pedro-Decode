package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Armored Autonomous (fr)", group = "Auto")
public class AutoWArtimisfr extends OpMode {

    public Follower follower;
    private int pathState;

    ElapsedTime delayTimer = new ElapsedTime();
    ElapsedTime autoTimer = new ElapsedTime();
    double delaySeconds = 5.0;
    final double AUTO_LENGTH_SECONDS = 30.0;
    final double AUTO_END_BUFFER_SECONDS = 1.0;

    LaunchSystem penguinsLauncher = new LaunchSystem();
    CameraSystem penguinsCamera = new CameraSystem();

    // Define important coordinate locations for the Blue side of the field
    private Pose startPose = new Pose(56, 8, 0);

    private Pose goUp = new Pose(56, 132, 0);
    private Pose shootPos = new Pose(44, 132, 0);
    private Pose Park = new Pose(58,132, 0);

    private PathChain goUpFr, launchPreload, ParkFr;


    @Override
    public void init() {

        // Mirror coordinates across the x-Axis if the autonomous is run on the Red side
        if (LocationChooser.chosenStartingLocation == LocationChooser.StartingLocation.RED_GOAL ||
                LocationChooser.chosenStartingLocation == LocationChooser.StartingLocation.RED_WALL) {
            startPose = startPose.mirror();
            goUp = goUp.mirror();
            shootPos = shootPos.mirror();
            Park = Park.mirror();
        }

        follower = Constants.createFollower(hardwareMap); // Make sure you create the follower before building paths
        buildPaths();
        follower.setStartingPose(startPose);

        // Initialize external systems
        penguinsLauncher.init(hardwareMap);
        penguinsCamera.init(hardwareMap);

        // Set the starting pose to null so that TeleOp will pick up with
        //  the Pinpoint position where this auto ends off
        LocationChooser.chosenStartingPose = null;
    }

    @Override
    public void init_loop() {

        // Modify the delay before the autonomous begins
        if (gamepad1.dpadUpWasPressed()) {
            delaySeconds += 0.5;
        }
        if (gamepad1.dpadDownWasPressed()) {
            delaySeconds -= 0.5;
        }
        telemetry.addData("Delay in seconds", delaySeconds);
        telemetry.addData("Location", LocationChooser.chosenStartingLocation);
        telemetry.update();

    }

    @Override
    public void start() {
        // Reset any timers
        delayTimer.reset();
        autoTimer.reset();

    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing - will also cause the robot to follow the current path
        penguinsLauncher.update();
        autonomousPathUpdate(); // Update autonomous state machine
    }

    public void buildPaths() {
        goUpFr = follower.pathBuilder()
                .addPath(new BezierLine(startPose, goUp))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPos.getHeading()).build();

        launchPreload = follower.pathBuilder()
                .addPath(new BezierLine(goUp, shootPos))
                .setLinearHeadingInterpolation(goUp.getHeading(), shootPos.getHeading()).build();

        ParkFr = follower.pathBuilder()
                .addPath(new BezierLine(shootPos, Park))
                .setLinearHeadingInterpolation(shootPos.getHeading(), Park.getHeading()).build();
    }

    public void autonomousPathUpdate() {

        // Autonomous state machine
        switch (pathState) {
            case 0:
                // Wait for the starting delay to expire
                if (delayTimer.seconds() > delaySeconds) {
                    // Begin the whole route
                        follower.followPath(goUpFr, true);
                        pathState = 1;
                }
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(launchPreload, true);
                    pathState = 2;
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    penguinsLauncher.fireBalls(3);
                    pathState = 3;
                }
                break;
            case 3:
                if (!penguinsLauncher.isBusy() && !follower.isBusy()) {
                    follower.followPath(ParkFr, true);
                    pathState = 999;
                }
                break;

        }
    }
}