package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import java.util.function.Supplier;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import java.text.DecimalFormat;

@TeleOp(name = "TeleOp", group = "OpModes")
public class CompetitionTeleOp extends OpMode {

    // Declare motors, servos, sensors, imus, etc.
    DcMotorEx motorLF, motorRF, motorLB, motorRB;
    GoBildaPinpointDriver pinpoint;
    RevColorSensorV3 colorSensor;
    Servo light1;

    // Create a telemetry manager so that telemetry shows up on the Panels dashboard
    TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

    // Create instances of any systems that need to be used
    LaunchSystem penguinsLauncher = new LaunchSystem();
    CameraSystem penguinsCamera = new CameraSystem();
    ColorSystem penguinsColorSensor = new ColorSystem();

    // PedroPathing follower declarations
    private Follower follower;
    private Supplier<PathChain> launchPath;  // Supplier is a functional interface in java that does not accept any parameters, but returns a value, using .get()
                                             //   This particular Supplier will return a PathChain and is implemented as a lamba function down below

    // Declare variables to store debug values
    int loopsOfFeederMotion;
    double flywheelErrorL;
    double flywheelErrorR;

    // Declare robot position variables
    double headingFieldCentric;
    double headingRadians;
    double robotX;
    double robotY;

    // Declare booleans about the robot's current mode/state
    boolean fieldCentricInUse = true;
    boolean isAutoDriving = false;
    boolean intakeIsActive = false;

    // Declare variables to store information from the LocationChooser file
    double driverHeadingDegrees;
    Pose startingPose;
    Pose launchPose;
    Pose goalPose;


    // Set Indicator Light constants
    public final double LED_RED = 0.279;
    public final double LED_BLUE = 0.611;
    public final double LED_GREEN = 0.5;
    public final double LED_YELLOW = 0.38;
    public final double LED_PURPLE = 0.7;

    double lightColor;
    public boolean isBlue;
    public boolean inPosition = false;
    public boolean inPositionPark = false;

    ColorSystem.SensedColors artifactColor = ColorSystem.SensedColors.UNKNOWN;


    // Create a decimal format for displaying values to telemetry with only a few decimal places visible
    DecimalFormat formatter = new DecimalFormat("#.###");

    // Custom class to store information for Z-Target Drive
    public class TargetHeading{
        // Instance variables
        private double directionMultiplier;
        private double errorDegrees;
        // Constructor
        public TargetHeading(double directionMultiplier, double errorDegrees) {
            this.directionMultiplier = directionMultiplier;
            this.errorDegrees = errorDegrees;
        }
        /** Get the direction multiplier, which is either +1.0 or -1.0 */
        public double getDirection() { return directionMultiplier; }
        /** Get the error degrees, which is how far the robot needs to turn to get to the target heading */
        public double getError() { return errorDegrees; }
    }

    TargetHeading towardsGoalHeading;



    // Runs once when INIT is pressed
    @Override
    public void init() {

        // Setup drive motors based on constants file
        motorLF = hardwareMap.get(DcMotorEx.class, Constants.driveConstants.leftFrontMotorName);
        motorLB = hardwareMap.get(DcMotorEx.class, Constants.driveConstants.leftRearMotorName);
        motorRF = hardwareMap.get(DcMotorEx.class, Constants.driveConstants.rightFrontMotorName);
        motorRB = hardwareMap.get(DcMotorEx.class, Constants.driveConstants.rightRearMotorName);

        // Setup other motors
        light1 = hardwareMap.servo.get("light1");

        // Reverse the motors based on constants file
        motorLF.setDirection(Constants.driveConstants.leftFrontMotorDirection);
        motorLB.setDirection(Constants.driveConstants.leftRearMotorDirection);
        motorRF.setDirection(Constants.driveConstants.rightFrontMotorDirection);
        motorRB.setDirection(Constants.driveConstants.rightRearMotorDirection);

        //This resets the encoder values when the code is initialized
        setDriveModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //This makes the wheels tense up and stay in position when it is not moving, opposite is FLOAT
        setDriveZeroPowerBehaviors(DcMotor.ZeroPowerBehavior.BRAKE);

        //This lets you look at encoder values while the OpMode is active
        //If you have a STOP_AND_RESET_ENCODER, make sure to put this below it
        setDriveModes(DcMotor.RunMode.RUN_USING_ENCODER);


        // Initialize external systems
        penguinsLauncher.init(hardwareMap);
        penguinsCamera.init(hardwareMap);
        penguinsColorSensor.init(hardwareMap);

        //follower = Constants.createFollower(hardwareMap);
        //follower.startTeleopDrive(true);

        // Pinpoint setup
        String pinpointName = Constants.localizerConstants.hardwareMapName;
        GoBildaPinpointDriver.EncoderDirection forwardDirection = Constants.localizerConstants.forwardEncoderDirection;
        GoBildaPinpointDriver.EncoderDirection strafeDirection = Constants.localizerConstants.strafeEncoderDirection;
        GoBildaPinpointDriver.GoBildaOdometryPods resolution = Constants.localizerConstants.encoderResolution;
        //  "xOffset" means the offset (Y) of the X (forward) pod   "forwardPodY" means the Y offset of the forward (X) pod
        double xOffset = Constants.localizerConstants.forwardPodY;
        //  "yOffset" means the offset (X) of the Y (strafe) pod   "strafePodX" means the X offset of the strafe (Y) pod
        double yOffset = Constants.localizerConstants.strafePodX;
        /*
                  +X
                   ^           Offsets are perpendicular to the tracking direction of the pod
             +Y <--|--> -Y          X is forward and Y is strafe
                   V                +X is up and +Y is left
                  -X
         */

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, pinpointName);
        pinpoint.setEncoderDirections(forwardDirection, strafeDirection);
        pinpoint.setOffsets(xOffset, yOffset, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(resolution);


        // Set poses and variables that depend upon the robot's starting location
        setLocationSpecificInformation();

        // Create a PedroPathing route to get to the launch position from anywhere on the field
        // Uses Lazy curve generation to allow for the route to change based on the robot's position in real time
        /*
        launchPath = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, launchPose)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, launchPose.getHeading(), 0.8))
                .build();
         */
            // This is the implementation of the Supplier from above
            //  It is a lambda function that returns a new PathChain, which is created on the fly using the robot's current pose

    }

    // Runs continuously after INIT is pressed and before START is pressed
    @Override
    public void init_loop() {

        // Display the chosen starting location just as confirmation for the drivers
        telemetryM.addData("Location", LocationChooser.chosenStartingLocation);
        telemetryM.update(telemetry);

    }

    // Runs once when START is pressed
    @Override
    public void start() {

        //
    }

    // Runs continually after START is pressed and before STOP is pressed
    @Override
    public void loop() {

        readFromPinpoint();

        // ....... MECANUM DRIVE CONTROLS .......
        // Set the desired powers based on joystick inputs (or dpad, for slow mode)
        double desiredForward = gamepad1.dpad_up ? 0.2 : (gamepad1.dpad_down ? -0.2 : -gamepad1.left_stick_y);
        double desiredStrafe = gamepad1.dpad_right ? 0.2 : (gamepad1.dpad_left ? -0.2 : gamepad1.left_stick_x);

        double powerAngular = -gamepad1.right_stick_x;
        double powerForward = desiredForward;  // Assume field-centric is not being used
        double powerStrafe = desiredStrafe;    // Assume field-centric is not being used

        // Modify powers based on robot heading for field-centric drive
        if (fieldCentricInUse) {
            powerForward = (desiredForward * Math.cos(headingFieldCentric)) - (desiredStrafe * Math.sin(headingFieldCentric));
            powerStrafe = (desiredStrafe * Math.cos(headingFieldCentric)) + (desiredForward * Math.sin(headingFieldCentric));
        }
        // Be able to toggle field centric drive in case the Pinpoint fails somehow
        if (gamepad1.startWasPressed()) {
            fieldCentricInUse = !fieldCentricInUse;
        }

        // Override angular power for Z-Target Drive
        if (gamepad1.a) {
            // Set target heading based on the goal and its position compared to the robot
            towardsGoalHeading = ZTargetCalculations(180);
            powerAngular = towardsGoalHeading.getDirection() * towardsGoalHeading.getError() / 30.0;
        }

        // Run the wheels using the desired powers
        if (!isAutoDriving) {
            mecanumDriveCode(powerForward, powerStrafe, powerAngular, 1.0);
        }



        if (!penguinsLauncher.isBusy()) {

            // ....... FLYWHEEL CONTROLS .......
            if (gamepad2.right_trigger > 0.5) {
                // Launch balls
                penguinsLauncher.setLauncherVelocity(penguinsLauncher.velocityRpm);
            } else if (gamepad2.left_trigger > 0.5) {
                // Reverse wheels to bring balls back in if necessary
                penguinsLauncher.setLauncherVelocity(-2500);
            } else {
                penguinsLauncher.setLauncherVelocity(0);
            }

            // ....... VELOCITY MODIFICATION .......
            if (gamepad2.dpadUpWasPressed()) {
                penguinsLauncher.changeTargetVelocity(50);
            } else if (gamepad2.dpadDownWasPressed()) {
                penguinsLauncher.changeTargetVelocity(-50);
            }
            telemetryM.addLine("Wheel Velocity: " + penguinsLauncher.velocityRpm + " RPM");


            // ....... INTAKE CONTROLS .......
            if (gamepad1.rightBumperWasPressed()) {
                // Switch intake states
                intakeIsActive = !intakeIsActive;
            }
            if (gamepad1.left_bumper) {
                // Left bumper overrides to outtake (in case of emergency)
                penguinsLauncher.setIntakePower(-1);
            } else if (intakeIsActive || gamepad2.left_bumper) {
                // Intake runs if it is toggled on
                penguinsLauncher.setIntakePower(1);
            } else {
                penguinsLauncher.setIntakePower(0);
            }

            // ....... FEEDER CONTROLS .......
            if (gamepad2.right_bumper) {
                penguinsLauncher.setFeederPosition(LaunchSystem.FEEDER_UP);
                loopsOfFeederMotion++;
            } else {
                penguinsLauncher.setFeederPosition(LaunchSystem.FEEDER_DOWN);
                loopsOfFeederMotion = 0;
            }

        }

        // ....... LAUNCH SEQUENCE MACROS .......
        if (gamepad2.yWasPressed()) {
            // Automatically launch one ball
            penguinsLauncher.fireBalls(1);
        }
        if (gamepad2.xWasPressed()) {
            // Automatically launch three sequential balls
            penguinsLauncher.fireBalls(3);
        }
        if (gamepad2.aWasPressed()) {
            // Emergency stop button to quit out of launching balls
            penguinsLauncher.stopActions();
        }
        penguinsLauncher.update();
        flywheelErrorL = penguinsLauncher.getFlywheelError(penguinsLauncher.launchL);
        flywheelErrorR = penguinsLauncher.getFlywheelError(penguinsLauncher.launchR);


        // ....... PEDRO PATHING FOLLOWER CODE .......
        // Only generate the path when X is first pressed down
        /*
        if (gamepad1.xWasPressed()) {
            follower.followPath(launchPath.get(), true);
        }
        // Continually follow the path for as long as X is held down
        if (gamepad1.x) {
            isAutoDriving = true;
            follower.update();
        } else {
            // Start PedroPathing TeleOp just to set the motors back to brake mode
            // By no longer calling .update(), we can still use our own TeleOp code
            if (isAutoDriving) follower.startTeleopDrive(true);
            isAutoDriving = false;
        }
        */


        // ....... LED LIGHT CODE .......
        if (isBlue) {
            lightColor = LED_BLUE;
        } else {
            lightColor = LED_RED;
        }
        light1.setPosition(lightColor);
        /*
        indicatorLightCode();

        artifactColor = penguinsColorSensor.getSensedColor();
        switch (artifactColor) {
            case PURPLE:
                light1.setPosition(LED_PURPLE);
                break;
            case GREEN:
                light1.setPosition(LED_GREEN);
                break;
            case UNKNOWN:
                light1.setPosition(0.0);
                break;
        }
         */


        //telemetry.addData("Label", "Information");

        telemetryM.addData("X Position", formatter.format( robotX ));
        telemetryM.addData("Y Position", formatter.format( robotY ));
        telemetryM.addData("Heading", formatter.format( Math.toDegrees(headingRadians) ));
        telemetryM.addData("Relative heading", formatter.format( Math.toDegrees(headingFieldCentric) ));

        telemetryM.addData("Loops since servo lifted", loopsOfFeederMotion);
        telemetryM.addData("Flywheel1 error", Math.round( flywheelErrorL ));
        telemetryM.addData("Flywheel2 error", Math.round( flywheelErrorR ));
        telemetryM.addData("Error difference", Math.round( flywheelErrorL - flywheelErrorR ));

        telemetryM.addData("Detected motif pattern", penguinsCamera.getHuskyLensPattern());
        telemetryM.update(telemetry);

    }


    /** Sets starting pose, goal pose, launch pose, alliance color, etc. using the results of the LocationChooser program */
    public void setLocationSpecificInformation() {

        // Set the alliance color (for the LED light)
        if (LocationChooser.chosenStartingLocation == LocationChooser.StartingLocation.BLUE_GOAL ||
                LocationChooser.chosenStartingLocation == LocationChooser.StartingLocation.BLUE_WALL) {
            isBlue = true;
        } else {
            isBlue = false;
        }

        // Import robot starting location from the Location Chooser, unless B is pressed (override for practice)
        if (gamepad1.b) {
            pinpoint.resetPosAndIMU();
            startingPose = new Pose(0, 0, 0);
            goalPose = startingPose;
            launchPose = startingPose;
            driverHeadingDegrees = 0;
        } else {
            startingPose = LocationChooser.chosenStartingPose;
            goalPose = LocationChooser.chosenGoalPose;
            launchPose = LocationChooser.chosenLaunchPose;
            driverHeadingDegrees = LocationChooser.chosenDriverHeading;
        }
        // Autonomous programs set startingPose to null
        if (startingPose != null) {
            pinpoint.setPosition(PoseConverter.poseToPose2D(startingPose, PedroCoordinates.INSTANCE));
        }
        // Set the follower's starting position based on the robot's current position, which is read from the Pinpoint
        pinpoint.update();
        //follower.setStartingPose(PoseConverter.pose2DToPose(pinpoint.getPosition(), PedroCoordinates.INSTANCE));
    }


    // Methods to easily set the attributes of all drive motors at once
    public void setDriveModes(DcMotor.RunMode mode) {
        motorLF.setMode(mode);
        motorLB.setMode(mode);
        motorRF.setMode(mode);
        motorRB.setMode(mode);
    }
    public void setDriveZeroPowerBehaviors(DcMotor.ZeroPowerBehavior behavior) {
        motorLF.setZeroPowerBehavior(behavior);
        motorLB.setZeroPowerBehavior(behavior);
        motorRF.setZeroPowerBehavior(behavior);
        motorRB.setZeroPowerBehavior(behavior);
    }

    /** Set the power of the drivetrain motors based on the desired powers in each direction
     * @param forward desired power in the forward direction
     * @param strafe desired power in the strafe direction
     * @param angular desired power for rotating'
     * @param speedPercent a value between 0 and 1 that is used to scale the final motor powers */
    public void mecanumDriveCode(double forward, double strafe, double angular, double speedPercent) {
        // Perform vector math to determine the desired powers for each wheel
        double powerLF = strafe + forward - angular;
        double powerLB = -strafe + forward - angular;
        double powerRF = -strafe + forward + angular;
        double powerRB = strafe + forward + angular;

        // Determine the greatest wheel power and set it to max
        double max = Math.max(1.0, Math.abs(powerLF));
        max = Math.max(max, Math.abs(powerRF));
        max = Math.max(max, Math.abs(powerLB));
        max = Math.max(max, Math.abs(powerRB));

        // Scale all power variables down to a number between 0 and 1 (so that setPower will accept them)
        motorLF.setPower(powerLF /max * speedPercent);
        motorLB.setPower(powerLB /max * speedPercent);
        motorRF.setPower(powerRF /max * speedPercent);
        motorRB.setPower(powerRB /max * speedPercent);
    }


    /** Update the robot's position and heading variables using the Pinpoint's data */
    public void readFromPinpoint() {
        pinpoint.update();
        headingRadians = pinpoint.getHeading(AngleUnit.RADIANS);
        headingFieldCentric = headingRadians - Math.toRadians(driverHeadingDegrees);
        robotX = pinpoint.getPosX(DistanceUnit.INCH);
        robotY = pinpoint.getPosY(DistanceUnit.INCH);
    }


    // ....... Z-TARGETING METHODS .......
    /** Performs all of the methods and operations needed to determine how the robot should turn to face towards the goal
     * @param headingOffset the offset, in degrees, that should be added to define the part of the robot that should face the goal (ex. an offset of 180
     * makes the back of the robot, where the launcher is, the new front that should face towards the goal
     * @return a TargetHeading object that tells how the robot should turn to point towards the goal */
    public TargetHeading ZTargetCalculations(double headingOffset) {
        double desiredHeadingDegrees = ratioOfSidesToHeading(goalPose.getX() - robotX,
                                                             goalPose.getY() - robotY);
        // By default, the intake is the front of the robot. To change that, an offset can be applied here
        return determineRotationDirection(Math.toDegrees(headingRadians) + headingOffset, desiredHeadingDegrees);
    }

    /** Performs an operation similar to atan2 that takes the two legs of a right triangle and determines the angle of the hypotenuse
     * @param X the length of the adjacent leg
     * @param Y the length of the opposite leg
     * @return the heading, in degrees, that the robot would need to have to be angled along the hypotenuse */
    public double ratioOfSidesToHeading(double X, double Y) {
        double freeHeading = 0.0;
        // If Y is zero, then the angle is purely horizontal
        // Otherwise, the angle can be calculated with trig
        if (Y == 0.0) {
            // Determine which horizontal based on the sign on X
            freeHeading = (X > 0.0) ? 0.0 : 180.0;
        } else {
            // Calculate the heading
            freeHeading = Math.toDegrees(Math.atan(Y/X));
        }
        /*
                                 With inverse tangent, (+X, +Y) and (-X, -Y) are indistinguishable
     135        90       45
            I   |   II
          -X,+Y | +X,+Y               inverse tangent accounts for quadrants I and II
     180 -------|-------  0      but can't tell the different between those and quadrants III and IV
          -X,-Y | +X,-Y
           III  |   IV           so, if Y is negative (quad III or IV), then add 180 degrees to
     225       270      315        the calculated angle

        */
        if (X < 0.0) {
            freeHeading += 180.0;
        } else if (Y < 0.0) {
            // Add additional code for quad IV to remove all negative values
            freeHeading += 360;
        }

        // Return the final calculated heading
        return freeHeading;
    }

    /** Determines the direction needed to turn for the robot to reach a desired heading
     * @return a TargetHeading object that specifics the direction needed to turn and the
     * difference in degrees between the desired heading and the current heading */
    public TargetHeading determineRotationDirection(double current, double target) {
        double currentCircularHeading = modPositive(current, 360);
        double targetHeading = modPositive(target, 360);
        double clockwiseDegrees;
        double counterclockwiseDegrees;

        // Determine the larger of the two headings
        if (targetHeading > currentCircularHeading) {
            // Subtract the smaller (current) heading from the larger (target) heading
            //   to find the degrees needed to turn to get to the target going counterclockwise
            counterclockwiseDegrees = targetHeading - currentCircularHeading;
            // Find the alternative
            clockwiseDegrees = 360 - counterclockwiseDegrees;
        } else {
            // Subtract the smaller (target) heading from the larger (current) heading
            //   to find the degrees needed to turn to get to the target doing clockwise
            clockwiseDegrees = currentCircularHeading - targetHeading;
            // Find the alternative
            counterclockwiseDegrees = 360 - clockwiseDegrees;
        }
        // Determine the most efficient direction and return the proper multiplier
        if (clockwiseDegrees < counterclockwiseDegrees) {
            return new TargetHeading(-1.0, Math.abs(clockwiseDegrees));
        } else {
            return new TargetHeading(1.0, Math.abs(counterclockwiseDegrees));
        }
    }

    /** Performs a mod operation that can only return positive results */
    public double modPositive(double number, double divisor) {
        return ((number % divisor) + divisor) % divisor;
    }


    // ....... LED LIGHT COLOR SELECTION LOGIC ........
    public void indicatorLightCode() {

        // Set the light color based on state
        if (inPosition) {
            lightColor = LED_GREEN;
        } else if (inPositionPark) {
           lightColor = LED_YELLOW;
        } else {
            if (isBlue) {
                lightColor = LED_BLUE;
            } else {
                lightColor = LED_RED;
            }
        }
        light1.setPosition(lightColor);

        // Toggle alliance color
        if (gamepad1.backWasPressed()) {
            isBlue = !isBlue;
        }

        // Determine whether the robot is in launching position (only red for now)
        //X Center: 35, Y Center: 38

        if (robotX > 30 && robotX < 34 && !isBlue) { // For Parking red
            inPositionPark = robotY > 37 && robotY < 47;

        }
        else if (robotX > 98 && robotX < 102 && isBlue) { // For Parking Blue
            inPositionPark = robotY < 40 && robotY > 30;
        }

        else if (robotX > 22.5 && robotX < 37.5 && isBlue) { // Blue Shooting
            inPosition = robotY > 120 && robotY < 130;
        }

        else if (robotX > 109.5 && robotX < 119.5 && !isBlue) { // Red Shooting
            inPosition = robotY > 115 && robotY < 125;
        }
        else {
            inPositionPark = false;
            inPosition = false;
        }
    }

} // end class