package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class AutoTest extends OpMode {

    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)

    LaunchSystem penguinsLauncher = new LaunchSystem();

    private final Pose startPose = new Pose(21, 122, Math.toRadians(324));
    private final Pose launchPose = new Pose(29, 116, Math.toRadians(324));
    private final Pose intakePose = new Pose(45, 104, Math.toRadians(324));
    private PathChain launchPath1, pickupPath, launchPath2;


    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap); // Make sure you create the follower before building paths
        buildPaths();
        follower.setStartingPose(startPose);

        penguinsLauncher.init(hardwareMap);
        AALocationChooser.chosenStartingPos = null;
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

        pickupPath = follower.pathBuilder()
            .addPath(new BezierLine(launchPose, intakePose))
            .setConstantHeadingInterpolation(intakePose.getHeading())
            .build();

        launchPath2 = follower.pathBuilder()
            .addPath(new BezierLine(intakePose, launchPose))
            .setConstantHeadingInterpolation(launchPose.getHeading())
            .build();
    }

    public void autonomousPathUpdate() {

        // Autonomous state machine
        switch (pathState) {
            case 0:
                // Begin the whole route
                follower.followPath(launchPath1, true);
                pathState = 1;
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
                    // Limit the max drivetrain power to 0.2 for slow intake sequence
                    penguinsLauncher.setIntakePower(1);
                    follower.followPath(pickupPath, 0.2, true);
                    pathState = 3;
                }
                break;
            case 3:
                /* Let the intake sequence play out */

                if (!follower.isBusy()) {
                    // Stop the intake and drive back to launch position
                    penguinsLauncher.setIntakePower(0);
                    follower.followPath(launchPath2, true);
                    pathState = 4;
                }
                break;
            case 4:
                /* Let the robot get back to launch position */

                if (!follower.isBusy()) {
                    // Begin the second launch sequence
                    penguinsLauncher.fireBalls(3);
                    pathState = 5;
                }
                break;
            case 5:
                /* Let the second launch sequence play out */

                if (!penguinsLauncher.isBusy()) {
                    // Quit out of the state machine and end the route
                    pathState = -1;
                }
                break;
        }
    }
}