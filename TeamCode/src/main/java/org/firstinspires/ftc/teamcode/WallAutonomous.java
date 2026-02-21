package org.firstinspires.ftc.teamcode;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.pedropathing.util.PoseHistory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Far-Side Wall Autonomous", group = "Auto")
public class WallAutonomous extends OpMode {

    public Follower follower;
    private int pathState;

    ElapsedTime delayTimer = new ElapsedTime();
    ElapsedTime autoTimer = new ElapsedTime();
    double delaySeconds = 0.0;
    final double AUTO_LENGTH_SECONDS = 30.0;
    final double AUTO_END_BUFFER_SECONDS = 1.0;

    LaunchSystem penguinsLauncher = new LaunchSystem();
    CameraSystem penguinsCamera = new CameraSystem();

    // Define important coordinate locations for the Blue side of the field
    private Pose startPose = new Pose(56, 9, 0);
    private Pose launchPose = new Pose(50, 14, Math.toRadians(290));

    private Pose intake1ControlPoint = new Pose(57, 38);
    private Pose intake1ReadyPose =  new Pose(42, 36, Math.toRadians(180));
    private Pose intake1FinishPose = new Pose(16, 36, Math.toRadians(180));

    private Pose leavePose = new Pose(43, 20, Math.toRadians(290));

    private PathChain launchPath1, intakePathReady1,intakePath1, launchPath2, leavePath;


    @Override
    public void init() {

        // Mirror coordinates across the x-Axis if the autonomous is run on the Red side
        if (LocationChooser.chosenStartingLocation == LocationChooser.StartingLocation.RED_GOAL ||
            LocationChooser.chosenStartingLocation == LocationChooser.StartingLocation.RED_WALL) {
            startPose = startPose.mirror();
            launchPose = launchPose.mirror();
            intake1ReadyPose = intake1ReadyPose.mirror();
            intake1ControlPoint = intake1ControlPoint.mirror();
            intake1FinishPose = intake1FinishPose.mirror();
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

        Drawing.init();
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
        penguinsLauncher.setTargetVelocity(penguinsLauncher.FAR_LAUNCH_VELOCITY);

    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing - will also cause the robot to follow the current path
        penguinsLauncher.update();
        autonomousPathUpdate(); // Update autonomous state machine

        telemetry.addData("Pose", follower.getPose());
        telemetry.update();
        Drawing.drawDebug(follower);
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

        // ....... Launch 2
        launchPath2 = follower.pathBuilder()
                .addPath(new BezierLine( intake1FinishPose, launchPose ))
                .setLinearHeadingInterpolation(intake1FinishPose.getHeading(), launchPose.getHeading()) // endTime denotes when the robot should be finished turning
                //.setTimeoutConstraint(150) // In milliseconds, defaults to 100
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
                    follower.followPath(intakePath1, 0.3, true);
                    pathState = 4;
                }
                break;
            case 4:
                /* Let the intake sequence play out */

                if (!follower.isBusy()) {
                    // Stop the intake and drive back to launch position
                    penguinsLauncher.setIntakePower(0);
                    penguinsLauncher.setLauncherVelocity(penguinsLauncher.velocityRpm);
                    follower.followPath(launchPath2, true);
                    pathState = 5;
                }
                break;
            case 5:
                /* Let the robot get back to launch position */

                if (!follower.isBusy()) {
                    // Begin the second launch sequence
                    penguinsLauncher.fireBalls(3, true);
                    pathState = 6;
                }
                break;
            case 6:
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



class Drawing {
    public static final double ROBOT_RADIUS = 9; // woah
    private static final FieldManager panelsField = PanelsField.INSTANCE.getField();

    private static final Style robotLook = new Style(
            "", "#3F51B5", 0.75
    );
    private static final Style historyLook = new Style(
            "", "#4CAF50", 0.75
    );

    /** This prepares Panels Field for using Pedro Offsets */
    public static void init() {
        panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
    }

    /**
     * This draws everything that will be used in the Follower's telemetryDebug() method. This takes
     * a Follower as an input, so an instance of the DashbaordDrawingHandler class is not needed.
     *
     * @param follower Pedro Follower instance.
     */
    public static void drawDebug(Follower follower) {
        if (follower.getCurrentPath() != null) {
            drawPath(follower.getCurrentPath(), robotLook);
            Pose closestPoint = follower.getPointFromPath(follower.getCurrentPath().getClosestPointTValue());
            drawRobot(new Pose(closestPoint.getX(), closestPoint.getY(), follower.getCurrentPath().getHeadingGoal(follower.getCurrentPath().getClosestPointTValue())), robotLook);
        }
        drawPoseHistory(follower.getPoseHistory(), historyLook);
        drawRobot(follower.getPose(), historyLook);

        sendPacket();
    }

    /**
     * This draws a robot at a specified Pose with a specified
     * look. The heading is represented as a line.
     *
     * @param pose  the Pose to draw the robot at
     * @param style the parameters used to draw the robot with
     */
    public static void drawRobot(Pose pose, Style style) {
        if (pose == null || Double.isNaN(pose.getX()) || Double.isNaN(pose.getY()) || Double.isNaN(pose.getHeading())) {
            return;
        }

        panelsField.setStyle(style);
        panelsField.moveCursor(pose.getX(), pose.getY());
        panelsField.circle(ROBOT_RADIUS);

        Vector v = pose.getHeadingAsUnitVector();
        v.setMagnitude(v.getMagnitude() * ROBOT_RADIUS);
        double x1 = pose.getX() + v.getXComponent() / 2, y1 = pose.getY() + v.getYComponent() / 2;
        double x2 = pose.getX() + v.getXComponent(), y2 = pose.getY() + v.getYComponent();

        panelsField.setStyle(style);
        panelsField.moveCursor(x1, y1);
        panelsField.line(x2, y2);
    }

    /**
     * This draws a robot at a specified Pose. The heading is represented as a line.
     *
     * @param pose the Pose to draw the robot at
     */
    public static void drawRobot(Pose pose) {
        drawRobot(pose, robotLook);
    }

    /**
     * This draws a Path with a specified look.
     *
     * @param path  the Path to draw
     * @param style the parameters used to draw the Path with
     */
    public static void drawPath(Path path, Style style) {
        double[][] points = path.getPanelsDrawingPoints();

        for (int i = 0; i < points[0].length; i++) {
            for (int j = 0; j < points.length; j++) {
                if (Double.isNaN(points[j][i])) {
                    points[j][i] = 0;
                }
            }
        }

        panelsField.setStyle(style);
        panelsField.moveCursor(points[0][0], points[0][1]);
        panelsField.line(points[1][0], points[1][1]);
    }

    /**
     * This draws all the Paths in a PathChain with a
     * specified look.
     *
     * @param pathChain the PathChain to draw
     * @param style     the parameters used to draw the PathChain with
     */
    public static void drawPath(PathChain pathChain, Style style) {
        for (int i = 0; i < pathChain.size(); i++) {
            drawPath(pathChain.getPath(i), style);
        }
    }

    /**
     * This draws the pose history of the robot.
     *
     * @param poseTracker the PoseHistory to get the pose history from
     * @param style       the parameters used to draw the pose history with
     */
    public static void drawPoseHistory(PoseHistory poseTracker, Style style) {
        panelsField.setStyle(style);

        int size = poseTracker.getXPositionsArray().length;
        for (int i = 0; i < size - 1; i++) {

            panelsField.moveCursor(poseTracker.getXPositionsArray()[i], poseTracker.getYPositionsArray()[i]);
            panelsField.line(poseTracker.getXPositionsArray()[i + 1], poseTracker.getYPositionsArray()[i + 1]);
        }
    }

    /**
     * This draws the pose history of the robot.
     *
     * @param poseTracker the PoseHistory to get the pose history from
     */
    public static void drawPoseHistory(PoseHistory poseTracker) {
        drawPoseHistory(poseTracker, historyLook);
    }

    /**
     * This tries to send the current packet to FTControl Panels.
     */
    public static void sendPacket() {
        panelsField.update();
    }
}