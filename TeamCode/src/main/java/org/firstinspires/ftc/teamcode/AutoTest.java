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

    private final Pose startPose = new Pose(25, 127, Math.toRadians(324));
    private final Pose launchPose = new Pose(33, 110, Math.toRadians(315));
    private final Pose intake1ReadyPose = new Pose(48, 84, Math.toRadians(180));
    private final Pose intake1FinishPose = new Pose(24, 84, Math.toRadians(180));
    private final Pose intake2ReadyPose = new Pose(48, 60, Math.toRadians(180));
    private final Pose intake2FinishPose = new Pose(24, 60, Math.toRadians(180));

    private PathChain launchPath1, intakePathReady1, intakePath1, launchPath2, intakePathReady2, intakePath2, launchPath3;


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
        // ....... Launch 1
        launchPath1 = follower.pathBuilder()
                .addPath(new BezierLine(  startPose, launchPose  ))
                .setConstantHeadingInterpolation(launchPose.getHeading()).build();

        // ....... Intake 1
        intakePathReady1 = follower.pathBuilder()
                .addPath(new BezierLine(  launchPose, intake1ReadyPose  ))
                .setLinearHeadingInterpolation(launchPose.getHeading(), intake1ReadyPose.getHeading()).build();
        intakePath1 = follower.pathBuilder()
                .addPath(new BezierLine(  intake1ReadyPose, intake1FinishPose  ))
                .setTangentHeadingInterpolation().build();

        // ....... Launch 2
        launchPath2 = follower.pathBuilder()
                .addPath(new BezierLine(  intake1FinishPose, launchPose  ))
                .setLinearHeadingInterpolation(intake1FinishPose.getHeading(), launchPose.getHeading()).build();

        // ....... Intake 2
        intakePathReady2 = follower.pathBuilder()
                .addPath(new BezierLine(  launchPose, intake2ReadyPose  ))
                .setLinearHeadingInterpolation(launchPose.getHeading(), intake2ReadyPose.getHeading()).build();
        intakePath2 = follower.pathBuilder()
                .addPath(new BezierLine(  intake2ReadyPose, intake2FinishPose  ))
                .setTangentHeadingInterpolation().build();

        // ....... Launch 3
        launchPath3 = follower.pathBuilder()
                .addPath(new BezierLine(  intake2FinishPose, launchPose  ))
                .setLinearHeadingInterpolation(intake2FinishPose.getHeading(), launchPose.getHeading()).build();
    }

    public void autonomousPathUpdate() {

        // Autonomous state machine
        switch (pathState) {
            case 0:
                // Wait for the starting delay to expire
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
                    follower.followPath(intakePathReady1);
                    pathState = 3;
                }
                break;
            case 3:
                /* Let the robot get to the first line of balls */

                if (!follower.isBusy()) {
                    // Intake the first line of balls
                    penguinsLauncher.setIntakePower(1);
                    follower.followPath(intakePath1, 0.2, true);
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
                    // Drive to the second line of balls
                    follower.followPath(intakePathReady2);
                    pathState = 7;
                }
                break;
            case 7:
                /* Let the robot get to the second line of balls */

                if (!follower.isBusy()) {
                    // Intake the second line of balls
                    penguinsLauncher.setIntakePower(1);
                    follower.followPath(intakePath2, 0.2, true);
                    pathState = 8;
                }
                break;
            case 8:
                /* Let the intake sequence play out */

                if (!follower.isBusy()) {
                    // Stop the intake and drive back to launch position
                    penguinsLauncher.setIntakePower(0);
                    follower.followPath(launchPath3, true);
                    pathState = 9;
                }
                break;
            case 9:
                /* Let the robot get back to launch position */

                if (!follower.isBusy()) {
                    // Begin the third launch sequence
                    penguinsLauncher.fireBalls(3);
                    pathState = 10;
                }
                break;
            case 10:
                /* Let the third launch sequence play out */

                if (!penguinsLauncher.isBusy()) {
                    // Quit out of the state machine and end the route
                    pathState = -1;
                }
                break;
        }
    }
}