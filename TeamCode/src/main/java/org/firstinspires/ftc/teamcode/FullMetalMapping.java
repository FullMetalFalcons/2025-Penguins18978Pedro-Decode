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

@Autonomous(name = "FullMetalMapping (auto with max fr)", group = "Auto")
public class FullMetalMapping extends OpMode {

    public Follower follower;
    private int pathState;

    ElapsedTime delayTimer = new ElapsedTime();
    ElapsedTime autoTimer = new ElapsedTime();
    ElapsedTime stateTimer = new ElapsedTime();
    double delaySeconds = 0.0;
    final double AUTO_LENGTH_SECONDS = 30.0;
    final double AUTO_END_BUFFER_SECONDS = 1.0;

    LaunchSystem penguinsLauncher = new LaunchSystem();
    CameraSystem penguinsCamera = new CameraSystem();

    // Define important coordinate locations for the Blue side of the field
    private Pose startPose = new Pose(56, 9, 0);
    private Pose launchPose = new Pose(50, 14, Math.toRadians(290));

    private Pose intake3ReadyPose = new Pose(42, 36, Math.toRadians(180));
    private Pose intake3FinishPose = new Pose(16, 36, Math.toRadians(180));

    private Pose intake2ReadyPose = new Pose(42, 60, Math.toRadians(180));
    private Pose intake2FinishPose = new Pose(18, 60, Math.toRadians(180));
    private Pose hitLever = new Pose(17, 76, Math.toRadians(90));
    private Pose hitLeverControlPoint = new Pose(32  , 80);

    private Pose leavePose = new Pose(43, 20, Math.toRadians(290));

    private PathChain launchPreload, intakePathReady3, intakePathFinish3, launchIntake3, intakePathReady2, intakePathFinish2, hitLeverFR, launchIntake2, leaveShootZone;


    @Override
    public void init() {

        // Mirror coordinates across the x-Axis if the autonomous is run on the Red side
        if (LocationChooser.chosenStartingLocation == LocationChooser.StartingLocation.RED_GOAL ||
                LocationChooser.chosenStartingLocation == LocationChooser.StartingLocation.RED_WALL) {
            startPose.mirror();
            launchPose.mirror();
            intake3ReadyPose.mirror();
            intake3FinishPose.mirror();
            //anal or oral
            intake2ReadyPose.mirror();
            intake2FinishPose.mirror();
            hitLever.mirror();
            hitLeverControlPoint.mirror();
            leavePose.mirror();
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
        telemetry.addData("Path State", pathState);
        telemetry.addData("State Timer", stateTimer);
        telemetry.update();

    }

    @Override
    public void start() {
        // Reset any timers
        delayTimer.reset();
        autoTimer.reset();
        penguinsLauncher.setTargetVelocity(3300);

    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing - will also cause the robot to follow the current path
        penguinsLauncher.update();
        autonomousPathUpdate(); // Update autonomous state machine
    }

    public void buildPaths() {
        // ....... Launch 1
        launchPreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, launchPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), launchPose.getHeading())
                .build();

        intakePathReady3 = follower.pathBuilder()
                .addPath(new BezierLine(launchPose, intake3ReadyPose))
                .setLinearHeadingInterpolation(launchPose.getHeading(), intake3ReadyPose.getHeading())
                .build();

        intakePathFinish3 = follower.pathBuilder()
                .addPath(new BezierLine(intake3ReadyPose, intake3FinishPose))
                .setLinearHeadingInterpolation(intake3ReadyPose.getHeading(), intake3FinishPose.getHeading())
                .build();

        launchIntake3 = follower.pathBuilder()
                .addPath(new BezierLine(intake3FinishPose, launchPose))
                .setLinearHeadingInterpolation(intake3FinishPose.getHeading(), launchPose.getHeading())
                .build();

        intakePathReady2 = follower.pathBuilder()
                .addPath(new BezierLine(launchPose, intake2ReadyPose))
                .setLinearHeadingInterpolation(launchPose.getHeading(), intake2ReadyPose.getHeading())
                .build();

        intakePathFinish2 = follower.pathBuilder()
                .addPath(new BezierLine(intake2ReadyPose, intake2FinishPose))
                .setLinearHeadingInterpolation(intake2ReadyPose.getHeading(), intake2FinishPose.getHeading())
                .build();

        hitLeverFR = follower.pathBuilder()
                .addPath(new BezierCurve(intake2FinishPose, hitLeverControlPoint, hitLever))
                .setLinearHeadingInterpolation(intake2FinishPose.getHeading(), hitLever.getHeading())
                .build();

        launchIntake2 = follower.pathBuilder()
                .addPath(new BezierLine(hitLever, launchPose))
                .setLinearHeadingInterpolation(hitLever.getHeading(), launchPose.getHeading())
                .build();

        leaveShootZone = follower.pathBuilder()
                .addPath(new BezierLine(launchPose, leavePose))
                .setLinearHeadingInterpolation(launchPose.getHeading(), leavePose.getHeading())
                .build();
    }

        public void autonomousPathUpdate() {

            // Autonomous state machine
            switch (pathState) {
                case 0:
                    // Wait for the starting delay to expire
                    if (delayTimer.seconds() > delaySeconds) {
                        // Begin the whole route
                        follower.followPath(launchPreload, true);
                        pathState = 1;
                    }
                    break;
                case 1:
                    /* Let the robot get to launch position */

                    if (!follower.isBusy()) {
                        // Begin the first launch sequence
                        penguinsLauncher.fireBalls(3);
                        pathState = 2;
                    }
                    break;
                case 2:
                    /* Let the first launch sequence play out */

                    if (!penguinsLauncher.isBusy()) {
                        // drive to the first line of balls
                        follower.followPath(intakePathReady3);
                        pathState = 3;
                    }
                    break;
                case 3:
                    /* Let the robot get to the first line of balls */

                    if (!follower.isBusy()) {
                        // Intake the first line of balls
                        penguinsLauncher.setIntakePower(1);
                        follower.followPath(intakePathFinish3, 0.5, true);
                        pathState = 4;
                    }
                    break;
                case 4:
                    /* Let the intake sequence play out */

                    if (!follower.isBusy()) {
                        // Stop the intake and drive back to launch position
                        penguinsLauncher.setIntakePower(0);
                        penguinsLauncher.setLauncherVelocity(penguinsLauncher.velocityRpm);
                        follower.followPath(launchIntake3, true);
                        pathState = 5;
                    }
                    break;
                case 5:
                    /* Let the robot get back to launch position */

                    if (!follower.isBusy()) {
                        // Begin the second launch sequence
                        penguinsLauncher.fireBalls(3, true);
                        pathState = 6;
                        stateTimer.reset();
                    }
                    break;

                case 6:
                    if (!penguinsLauncher.isBusy() && stateTimer.seconds() > 5) {
                        follower.followPath(intakePathReady2);
                        pathState = 7;
                    }
                    break;
                case 7:
                    if (!follower.isBusy()) {
                        penguinsLauncher.setIntakePower(1);
                        follower.followPath(intakePathFinish2);
                        pathState = 8;
                    }
                    break;

                case 8:
                    if (!follower.isBusy()) {
                        penguinsLauncher.setIntakePower(0);
                        follower.followPath(hitLeverFR);
                        pathState = 9;
                        stateTimer.reset();
                    }
                    break;
                case 9:
                    if (!follower.isBusy() && stateTimer.seconds() > 2.5) {
                        follower.followPath(launchIntake2);
                        pathState = 10;
                    }
                    break;
                case 10:
                    if (!follower.isBusy()) {
                        penguinsLauncher.fireBalls(3);
                        pathState = 11;
                    }
                    break;
                case 11:
                    if (!penguinsLauncher.isBusy()) {
                        follower.followPath(leaveShootZone);
                        pathState = -1;
                    }
                    break;
/*
            case 999:
                 Let the third launch sequence play out

                // If the launch sequence is finished, or autonomous is about to end, move sideways for the Leave points
                if (autoTimer.seconds() > AUTO_LENGTH_SECONDS - AUTO_END_BUFFER_SECONDS
                        || !penguinsLauncher.isBusy()) {

                    // Quit out of the state machine and move off of the Launch line
                    follower.followPath(leavePath, true);
                    pathState = -1;
                }
                break;
                */
            }
        }
    }