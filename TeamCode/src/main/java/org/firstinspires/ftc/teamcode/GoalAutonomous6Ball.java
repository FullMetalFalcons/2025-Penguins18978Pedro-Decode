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

@Autonomous(name = "6-GoalAutonomous", group = "Auto")
public class GoalAutonomous6Ball extends OpMode {

    public Follower follower;
    private int pathState;

    ElapsedTime delayTimer = new ElapsedTime();
    ElapsedTime autoTimer = new ElapsedTime();
    boolean shouldOpenGate = true;
    double delaySeconds = 0.0;
    final double AUTO_LENGTH_SECONDS = 30.0;
    final double AUTO_END_BUFFER_SECONDS = 1.0;

    LaunchSystem penguinsLauncher = new LaunchSystem();
    CameraSystem penguinsCamera = new CameraSystem();

    // Define important coordinate locations for the Blue side of the field
    private Pose startPose = new Pose(18, 113, 0);
    private Pose launchPose = LocationChooser.BLUE_LAUNCH_POSE;

    private Pose intake1ControlPoint = new Pose(48, 104);
    private Pose intake1ReadyPose =  new Pose(44, 84, Math.toRadians(180));
    private Pose intake1FinishPose = new Pose(18, 84, Math.toRadians(180));

    private Pose hitLeverPose = new Pose(17, 75, Math.toRadians(180));
    private Pose hitLeverControlPoint = new Pose(40, 80);

    private Pose intake2ControlPoint = new Pose(51, 107);
    private Pose intake2ReadyPose =  new Pose(44, 60, Math.toRadians(180));
    private Pose intake2FinishPose = new Pose(18, 60, Math.toRadians(180));

    private Pose launch3ControlPoint = new Pose(55, 58);

    private Pose intake3ReadyPose = new Pose(44, 36, Math.toRadians(180));
    private Pose intake3FinishPose = new Pose(18, 36, Math.toRadians(180));

    private Pose launch4ControlPoint = new Pose(55, 58);
    private Pose leavePose = new Pose(45, 113, Math.toRadians(315));

    private PathChain launchPath1, intakePathReady1,intakePath1, launchPath2, intakePathReady2,intakePath2, launchPath3, intakePathReady3, intakePath3, launchPath4, leavePath, hitLever1;


    @Override
    public void init() {

        // Mirror coordinates across the x-Axis if the autonomous is run on the Red side
        if (LocationChooser.chosenStartingLocation == LocationChooser.StartingLocation.RED_GOAL ||
                LocationChooser.chosenStartingLocation == LocationChooser.StartingLocation.RED_WALL) {
            startPose = startPose.mirror();
            launchPose = LocationChooser.RED_LAUNCH_POSE;
            intake1ReadyPose = intake1ReadyPose.mirror();
            intake1ControlPoint = intake1ControlPoint.mirror();
            intake1FinishPose = intake1FinishPose.mirror();
            intake2ReadyPose = intake2ReadyPose.mirror();
            intake2ControlPoint = intake2ControlPoint.mirror();
            intake2FinishPose = intake2FinishPose.mirror();
            launch3ControlPoint = launch3ControlPoint.mirror();
            hitLeverPose = hitLeverPose.mirror();
            hitLeverControlPoint = hitLeverControlPoint.mirror();
            intake3ReadyPose = intake3ReadyPose.mirror();
            intake3FinishPose = intake3FinishPose.mirror();
            launch4ControlPoint = launch4ControlPoint.mirror();
            leavePose = leavePose.mirror();
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
        // Choose whether or not to go for opening the gate
        if (gamepad1.dpadRightWasPressed()) {
            shouldOpenGate = !shouldOpenGate;
        }
        telemetry.addData("Delay in seconds", delaySeconds);
        telemetry.addData("Open Gate", shouldOpenGate);
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
        telemetry.addData("IsBusy", follower.isBusy());
        telemetry.addData("State", pathState);
        telemetry.addData("Timer", delayTimer.seconds());
    }

    public void buildPaths() {
        // ....... Launch 1
        launchPath1 = follower.pathBuilder()
                .addPath(new BezierLine(  startPose, launchPose  ))
                .setLinearHeadingInterpolation(startPose.getHeading(), launchPose.getHeading()).build();

        // ....... Intake 1
        intakePathReady1 = follower.pathBuilder()
                .addPath(new BezierCurve(  launchPose, intake1ControlPoint, intake1ReadyPose  ))
                .setLinearHeadingInterpolation(launchPose.getHeading(), intake1ReadyPose.getHeading()).build();
        intakePath1 = follower.pathBuilder()
                .addPath(new BezierLine(  intake1ReadyPose, intake1FinishPose  ))
                .setTangentHeadingInterpolation().build();
        hitLever1 = follower.pathBuilder()
                .addPath(new BezierCurve(  intake1FinishPose, hitLeverControlPoint, hitLeverPose  ))
                .setConstantHeadingInterpolation(hitLeverPose.getHeading())
                .build();

        // ....... Launch 2
        launchPath2 = follower.pathBuilder()
                .addPath(new BezierLine(  intake1FinishPose, launchPose  ))
                .setLinearHeadingInterpolation(intake1FinishPose.getHeading(), launchPose.getHeading()).build();

        // ....... Intake 2
        intakePathReady2 = follower.pathBuilder()
                .addPath(new BezierCurve(  launchPose, intake2ControlPoint, intake2ReadyPose  ))
                .setLinearHeadingInterpolation(launchPose.getHeading(), intake2ReadyPose.getHeading()).build();
        intakePath2 = follower.pathBuilder()
                .addPath(new BezierLine(  intake2ReadyPose, intake2FinishPose  ))
                .setTangentHeadingInterpolation().build();

        // ....... Launch 3
        launchPath3 = follower.pathBuilder()
                .addPath(new BezierCurve(  intake2FinishPose, launch3ControlPoint, launchPose  ))
                .setLinearHeadingInterpolation(intake2FinishPose.getHeading(), launchPose.getHeading()).build();

        // ....... Intake 3
        intakePathReady3 = follower.pathBuilder()
                .addPath(new BezierLine(launchPose, intake3ReadyPose))
                .setLinearHeadingInterpolation(launchPose.getHeading(), intake3ReadyPose.getHeading())
                .build();
        intakePath3 = follower.pathBuilder()
                .addPath(new BezierLine(intake3ReadyPose, intake3FinishPose))
                .setLinearHeadingInterpolation(intake3ReadyPose.getHeading(), intake3FinishPose.getHeading())
                .build();

        // ....... Launch 4
        launchPath4 = follower.pathBuilder()
                .addPath(new BezierCurve(intake3FinishPose, launch4ControlPoint, launchPose))
                .setLinearHeadingInterpolation(intake3FinishPose.getHeading(), launchPose.getHeading())
                .build();

        // ....... Leave Points
        leavePath = follower.pathBuilder()
                .addPath(new BezierLine(  launchPose, leavePose  ))
                .setConstantHeadingInterpolation(leavePose.getHeading())
                .build();
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
                    follower.followPath(intakePath1, 0.4, true);
                    pathState = 4;
                }
                break;
            case 4:
                /* Let the intake sequence play out */

                if (!follower.isBusy()) {
                    if (shouldOpenGate) {
                        follower.followPath(hitLever1);
                        delayTimer.reset();
                        pathState = 5;
                    } else {
                        pathState = 6;
                    }
                }
                break;
            case 5:
                /* Pause for the gate sequence */

                // TODO: Fix bad coding - The delay should begin after the follower finishes moving the robot
                if (!follower.isBusy() && delayTimer.seconds() > 3) {
                    pathState = 6;
                }
                break;
            case 6:
                /* Let the robot open the gate, if it needs to */

                if (!follower.isBusy()) {
                    // Stop the intake and drive back to launch position
                    penguinsLauncher.setIntakePower(0);
                    //penguinsLauncher.launcherPrepared = true;
                    penguinsLauncher.setLauncherVelocity(penguinsLauncher.velocityRpm);
                    follower.followPath(launchPath2, true);
                    pathState = 7;
                }
                break;
            case 7:
                /* Let the robot get back to launch position */

                if (!follower.isBusy()) {
                    // Begin the second launch sequence
                    penguinsLauncher.fireBalls(3, true);
                    pathState = 16; // Skip to the end because we only want to shoot 6 balls
                }
                break;
//            case 8:
//                /* Let the second launch sequence play out */
//
//                if (!penguinsLauncher.isBusy()) {
//                    // Drive to the second line of balls
//                    follower.followPath(intakePathReady2);
//                    pathState = 9;
//                }
//                break;
//            case 9:
//                /* Let the robot get to the second line of balls */
//
//                if (!follower.isBusy()) {
//                    // Intake the second line of balls
//                    penguinsLauncher.setIntakePower(1);
//                    follower.followPath(intakePath2, 0.4, true);
//                    pathState = 10;
//                }
//                break;
//            case 10:
//                /* Let the intake sequence play out */
//
//                if (!follower.isBusy()) {
//                    // Stop the intake and drive back to launch position
//                    penguinsLauncher.setIntakePower(0);
//                    //penguinsLauncher.launcherPrepared = true;
//                    penguinsLauncher.setLauncherVelocity(penguinsLauncher.velocityRpm);
//                    follower.followPath(launchPath3, true);
//                    pathState = 11;
//                }
//                break;
//            case 11:
//                /* Let the robot get back to launch position */
//
//                if (!follower.isBusy()) {
//                    // Begin the third launch sequence
//                    penguinsLauncher.fireBalls(3, true);
//                    pathState = 12;
//                }
//                break;
//            case 12:
//                /* Let the robot get to the third line of balls */
//
//                if (!penguinsLauncher.isBusy()) {
//                    penguinsLauncher.setIntakePower(0);
//                    follower.followPath(intakePathReady3, true);
//                    pathState = 13;
//                }
//                break;
//            case 13:
//                /* Let the intake sequence play out */
//
//                if (!follower.isBusy()) {
//                    penguinsLauncher.setIntakePower(1);
//                    follower.followPath(intakePath3, 0.4, true);
//                    pathState = 14;
//                }
//                break;
//            case 14:
//                if (!follower.isBusy()) {
//                    penguinsLauncher.setIntakePower(0);
//                    //penguinsLauncher.launcherPrepared = true;
//                    penguinsLauncher.setLauncherVelocity(penguinsLauncher.velocityRpm);
//                    follower.followPath(launchPath4, true);
//                    pathState = 15;
//                }
//                break;
//            case 15:
//                if (!follower.isBusy()) {
//                    penguinsLauncher.fireBalls(3, true);
//                    pathState = 16;
//                }
//                break;

            case 16:
                /* Let the third launch sequence play out */

                // If the launch sequence is finished, or autonomous is about to end, move sideways for the Leave points
                if (autoTimer.seconds() > AUTO_LENGTH_SECONDS - AUTO_END_BUFFER_SECONDS
                        || !penguinsLauncher.isBusy()) {

                    // Quit out of the state machine and move off of the Launch line
                    follower.followPath(leavePath, true);
                    pathState = -1;
                }
                break;
        }
    }
}