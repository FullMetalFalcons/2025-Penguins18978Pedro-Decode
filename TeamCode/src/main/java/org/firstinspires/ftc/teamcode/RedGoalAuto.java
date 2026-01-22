package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class RedGoalAuto extends OpMode {

    LaunchSystem penguinsLauncher = new LaunchSystem();

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private final Pose startPose = new Pose(119.128, 127.346, Math.toRadians(-143));
    private final Pose parkPos = new Pose(105.121, 33.401, Math.toRadians(90));
    private final Pose shootPos = new Pose(103.066, 112.778, Math.toRadians(40));
    private final Pose firstRowStart = new Pose(100.265, 83.268, Math.toRadians(0));
    private final Pose firstRowEnd = new Pose(128.840, 83.829, Math.toRadians(0));
    private final Pose secondRowStart = new Pose(100.265, 59.736, Math.toRadians(0));
    private final Pose secondRowEnd = new Pose(128.840, 59.736, Math.toRadians(0));
    private final Pose thirdRowStart = new Pose(100.265, 35.455, Math.toRadians(0));
    private final Pose thirdRowEnd = new Pose(128.840, 35.455, Math.toRadians(0));

    private int pathState;

    private Path scorePreload;
    private PathChain grabPickup1, pickupPickup1, scorePickup1, grabPickup2,  pickupPickup2, scorePickup2, grabPickup3, pickupPickup3, scorePickup3, park;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, shootPos));

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPos, firstRowStart))
                .setLinearHeadingInterpolation(shootPos.getHeading(), firstRowStart.getHeading())
                .build();

        pickupPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(firstRowStart, firstRowEnd))
                .setLinearHeadingInterpolation(firstRowStart.getHeading(), firstRowEnd.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(firstRowEnd, shootPos))
                .setLinearHeadingInterpolation(firstRowEnd.getHeading(), shootPos.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPos, secondRowStart))
                .setLinearHeadingInterpolation(shootPos.getHeading(), secondRowStart.getHeading())
                .build();

        pickupPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(secondRowStart, secondRowEnd))
                .setLinearHeadingInterpolation(secondRowStart.getHeading(), secondRowEnd.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(secondRowEnd, shootPos))
                .setLinearHeadingInterpolation(secondRowEnd.getHeading(), shootPos.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(shootPos, thirdRowStart))
                .setLinearHeadingInterpolation(shootPos.getHeading(), thirdRowStart.getHeading())
                .build();

        pickupPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(thirdRowStart, thirdRowEnd))
                .setLinearHeadingInterpolation(thirdRowStart.getHeading(), thirdRowEnd.getHeading())
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(thirdRowEnd, shootPos))
                .setLinearHeadingInterpolation(thirdRowEnd.getHeading(), shootPos.getHeading())
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(shootPos, parkPos))
                .setLinearHeadingInterpolation(shootPos.getHeading(), parkPos.getHeading())
                .build();
    }

    public void autoPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup1);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup1);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(pickupPickup1);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup1);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup2);
                    setPathState(6);
                }
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(pickupPickup2);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup2);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup3);
                    setPathState(9);
                }
            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(pickupPickup3);
                    setPathState(10);
                }
            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup3);
                    setPathState(11);
                }
            case 11:
                if (!follower.isBusy()) {
                    follower.followPath(park);
                    setPathState(-1);
                }
                break;

        }
    }



    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        penguinsLauncher.init(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void loop() {
        follower.update();
        penguinsLauncher.update();
        autoPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();

    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}