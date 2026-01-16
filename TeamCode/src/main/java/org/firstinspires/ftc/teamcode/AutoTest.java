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
        follower.setStartingPose(new Pose(72, 72, Math.toRadians(180)));

        paths = new Paths(follower); // Build paths
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing - will also cause the robot to follow the current path
        pathState = autonomousPathUpdate(); // Update autonomous state machine
    }

    public static class Paths {

        public PathChain forward;
        public PathChain strafe;
        public PathChain curve;

        public Paths(Follower follower) {
            forward = follower.pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(72.000, 72.000), new Pose(48.116, 72.000))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            strafe = follower.pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(48.116, 72.000), new Pose(48.291, 47.942))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270))
                    .build();

            curve = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(48.291, 47.942),
                                    new Pose(48.116, 69.385),
                                    new Pose(72.349, 72.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(180))
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
                // Start the "forward" path and change states
                follower.followPath(paths.forward);
                pathState = 1;
                break;

            case 1:
                // When the "forward" path finishes, move to the next path
                if (!follower.isBusy()) {
                    follower.followPath(paths.strafe);
                    pathState = 2;
                }
                break;

            case 2:
                // When the "strafe" path finishes, move to the next path
                if (!follower.isBusy()) {
                    follower.followPath(paths.curve, true);
                    pathState = 3;
                }
                break;

            case 3:
                // When the "curve" path finishes, the auto is done, so end the state machine
                if (!follower.isBusy()) {
                    pathState = -1;
                }
                break;
        }

        // Return the new path state
        return pathState;
    }
}