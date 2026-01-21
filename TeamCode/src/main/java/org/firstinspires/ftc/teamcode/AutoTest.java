package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class AutoTest extends OpMode {

    public Follower follower; // Pedro Pathinsg follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 72, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing - will also cause the robot to follow the current path
        pathState = autonomousPathUpdate(); // Update autonomous state machine
    }

    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(72.000, 72.000),
                            new Pose(96.000, 96.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(90))
                    .build();

            Path2 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(96.000, 96.000),
                            new Pose(105.847, 107.776),
                            new Pose(106.649, 60.903)
                    )
                    ).setConstantHeadingInterpolation(Math.toRadians(90))
                    .build();

            Path3 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(106.649, 60.903),
                            new Pose(108.571, 93.499),
                            new Pose(71.053, 94.700),
                            new Pose(72.048, 71.683)
                    )
                    ).setTangentHeadingInterpolation()
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        /* Add your state machine Here
           Access paths with paths.pathName
           Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
         */

        switch (pathState) {
            case 0:
                follower.followPath(paths.Path1);
                pathState = 1;
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path2);
                    pathState = 2;
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path3, true);
                    pathState = 3;
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    pathState = -1;
                }
                break;
        }

        // Return the new path state
        return pathState;
    }
}