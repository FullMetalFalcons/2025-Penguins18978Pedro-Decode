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

@Autonomous
public class AutoTest extends OpMode {

    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)

    ElapsedTime delayTimer = new ElapsedTime();
    double delaySeconds = 0.0;

    LaunchSystem penguinsLauncher = new LaunchSystem();

    private final Pose startPose = new Pose(21, 122, Math.toRadians(324));
    private final Pose launchPose = new Pose(29, 116, Math.toRadians(324));
    private final Pose intake1ReadyPose = new Pose(43, 84, Math.toRadians(180));
    private final Pose intake1FinishPose = new Pose(24, 84, Math.toRadians(180));
    private PathChain launchPath1, pickupPathReady1, pickupPath1, launchPath2;


    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap); // Make sure you create the follower before building paths
        buildPaths();
        follower.setStartingPose(startPose);

        penguinsLauncher.init(hardwareMap);
        AALocationChooser.chosenStartingPos = null;
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
        telemetry.update();

    }

    @Override
    public void start() {
        // Reset the timer for the initial delay
        delayTimer.reset();

    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing - will also cause the robot to follow the current path
        penguinsLauncher.update();
        autonomousPathUpdate(); // Update autonomous state machine
    }

    public void buildPaths() {

        launchPath1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, launchPose))
                .setConstantHeadingInterpolation(launchPose.getHeading())
                .build();

        pickupPathReady1 = follower.pathBuilder()
                .addPath(new BezierCurve(launchPose,
                                         new Pose(72,84),
                                         intake1ReadyPose))
                .setLinearHeadingInterpolation(launchPose.getHeading(), intake1ReadyPose.getHeading())
                .build();

        pickupPath1 = follower.pathBuilder()
                .addPath(new BezierLine(intake1ReadyPose, intake1FinishPose))
                .setConstantHeadingInterpolation(intake1FinishPose.getHeading())
                .build();

        launchPath2 = follower.pathBuilder()
                .addPath(new BezierCurve(intake1FinishPose,
                                         new Pose(42,103),
                                         launchPose))
                .setLinearHeadingInterpolation(intake1FinishPose.getHeading(), launchPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {

        // Autonomous state machine
        switch (pathState) {
            case 0:
                if (delayTimer.seconds() > delaySeconds) {
                    // Begin the whole route
                    follower.followPath(launchPath1, true);
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
                    follower.followPath(pickupPathReady1);
                    pathState = 3;
                }
                break;
            case 3:
                /* Let the robot get to the first line of balls */

                if (!follower.isBusy()) {
                    // Intake the first line of balls
                    penguinsLauncher.setIntakePower(1);
                    follower.followPath(pickupPath1, 0.2, true);
                    pathState = 4;
                }

                break;
            case 4:
                /* Let the intake sequence play out */

                if (!follower.isBusy()) {
                    // Stop the intake and drive back to launch position
                    penguinsLauncher.setIntakePower(0);
                    follower.followPath(launchPath2, true);
                    pathState = 5;
                }
                break;
            case 5:
                /* Let the robot get back to launch position */

                if (!follower.isBusy()) {
                    // Begin the second launch sequence
                    penguinsLauncher.fireBalls(3);
                    pathState = 6;
                }
                break;
            case 6:
                /* Let the second launch sequence play out */

                if (!penguinsLauncher.isBusy()) {
                    // Quit out of the state machine and end the route
                    pathState = -1;
                }
                break;
        }
    }
}