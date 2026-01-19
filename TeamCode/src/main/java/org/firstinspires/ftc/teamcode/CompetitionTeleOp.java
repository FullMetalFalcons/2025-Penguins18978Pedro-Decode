package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

@TeleOp(name = "TeleOp", group = "OpModes")
@Configurable
public class CompetitionTeleOp extends OpMode {

    // Declare motors, servos, sensors, imus, etc.
    DcMotorEx motorLF, motorRF, motorLB, motorRB, intake, launchL, launchR;
    GoBildaPinpointDriver pinpoint;
    NormalizedColorSensor colorSensor;
    Servo feeder, light1, light2;

    // Create a telemetry manager so that telemetry shows up on the Panels dashboard
    TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

    // Create constants
    final double TICKS_PER_ROTATION = 28;
    final double TPS_PER_RPM = TICKS_PER_ROTATION / 60;
    /*
        Rotation       Tick       Minute
        --------  *  --------  *  ------
         Minute      Rotation     Second
     */

    public static double FEEDER_DOWN = 0.37; // static so that it can be tuned via Panels
    public static double FEEDER_UP = 0.47;
    int loopsOfFeederMotion;
    double flywheelErrorL;
    double flywheelErrorR;

    // Declare and/or initialize other variables
    int velocityRPM = 3200;

    public static double flywheelF = 13.0;
    public static double flywheelP = 200.0;

    boolean intakeIsActive;

    double headingFieldCentric;
    double headingRadians;
    double robotX;
    double robotY;
    boolean fieldCentricInUse = true;

    double driverHeadingDegrees;
    Pose2D startingPos;
    Pose2D goalPos;

    // Indicator Light constants
    public final double LED_RED = 0.279;
    public final double LED_BLUE = 0.611;
    public final double LED_GREEN = 0.5;
    public final double LED_YELLOW = 0.38;

    double lightColor;
    public boolean isBlue;
    public boolean inPosition = false;
    public boolean inPositionPark = false;

    // Color sensor variables
    public enum SensedColors {
        PURPLE,
        GREEN,
        UNKNOWN
    }
    public static double colorSensorGain = 5; // static so that it can be tuned via Panels


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
        intake = (DcMotorEx) hardwareMap.dcMotor.get("intake");
        launchL = (DcMotorEx) hardwareMap.dcMotor.get("launchL");
        launchR = (DcMotorEx) hardwareMap.dcMotor.get("launchR");
        feeder = hardwareMap.servo.get("feeder");
        light1 = hardwareMap.servo.get("light1");
        light2 = hardwareMap.servo.get("light2");

        // Set motor properties for the operator motors
        launchR.setDirection(DcMotorSimple.Direction.REVERSE);
        launchL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launchR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Set flywheel Feedforward and Proportional constants
        launchL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(flywheelP,0,0,  flywheelF));
        launchR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(flywheelP,0,0,  flywheelF));

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


        // Color sensor setup
        colorSensor = hardwareMap.get(NormalizedColorSensor.class,"color_sensor");


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

    }

    // Runs continuously after INIT is pressed and before START is pressed
    @Override
    public void init_loop() {

        // Display the chosen starting location just as confirmation for the drivers
        telemetryM.addData("Location", AALocationChooser.chosenStartingLocation);
        telemetryM.update(telemetry);

    }

    // Runs once when START is pressed
    @Override
    public void start() {

        // Set the alliance color (for the LED light)
        if (AALocationChooser.chosenStartingLocation == AALocationChooser.StartingLocation.BLUE_GOAL ||
                AALocationChooser.chosenStartingLocation == AALocationChooser.StartingLocation.BLUE_WALL) {
            isBlue = true;
        } else {
            isBlue = false;
        }

        // Import robot starting location from the Location Chooser, unless B is pressed (override for practice)
        if (gamepad1.b) {
            startingPos = new Pose2D(DistanceUnit.INCH, 0, 0,
                                     AngleUnit.DEGREES, 0);
            goalPos = startingPos;
            driverHeadingDegrees = 0;
        } else {
            startingPos = AALocationChooser.chosenStartingPos;
            goalPos = AALocationChooser.chosenGoalPos;
            driverHeadingDegrees = AALocationChooser.chosenDriverHeading;
        }
        // Autonomous programs set startingPos to null
        if (startingPos != null) {
            pinpoint.setPosition(startingPos);
        }
    }

    // Runs continually after START is pressed and before STOP is pressed
    @Override
    public void loop() {

        readFromPinpoint();

        // ....... MECANUM DRIVE CONTROLS .......
        // Set the desired powers based on joystick inputs (-1 to 1)
        double desiredForward = -gamepad1.left_stick_y;
        double desiredStrafe = gamepad1.left_stick_x;

        double powerAngular = -gamepad1.right_stick_x;
        double powerForward = desiredForward;  // Assume field centric is not being used
        double powerStrafe = desiredStrafe;    // Assume field centric is not being used

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
            powerAngular = towardsGoalHeading.getDirection() * towardsGoalHeading.getError()/30.0;
        }

        // Run the wheels using the desired powers
        mecanumDriveCode(powerForward, powerStrafe, powerAngular, 1.0);


        // ....... FLYWHEEL CONTROLS .......
        if (gamepad2.right_trigger > 0.5) {
            // Launch balls
            launchL.setVelocity(velocityRPM * TPS_PER_RPM);
            launchR.setVelocity(velocityRPM * TPS_PER_RPM);
        } else if (gamepad2.left_trigger > 0.5) {
            // Reverse wheels to bring balls back in if necessary
            launchL.setVelocity(-2500 * TPS_PER_RPM);
            launchR.setVelocity(-2500 * TPS_PER_RPM);
        } else {
            launchL.setVelocity(0);
            launchR.setVelocity(0);
        }
        flywheelErrorL = (velocityRPM * TPS_PER_RPM) - launchL.getVelocity();
        flywheelErrorR = (velocityRPM * TPS_PER_RPM) - launchR.getVelocity();

        // ....... VELOCITY MODIFICATION .......
        if (gamepad2.dpadUpWasPressed()) {
            velocityRPM += 100;
        } else if (gamepad2.dpadDownWasPressed()) {
            velocityRPM -= 100;
        }
        telemetryM.addLine("Wheel Velocity: " + velocityRPM + " RPM");


        // ....... INTAKE CONTROLS .......
        if (gamepad1.rightBumperWasPressed()) {
            // Switch intake states
            intakeIsActive = !intakeIsActive;
        }
        if (gamepad1.left_bumper) {
            // Left bumper overrides to outtake (in case of emergency)
            intake.setPower(-1);
        } else if (intakeIsActive || gamepad2.left_bumper) {
            // Intake runs if it is toggled on
            intake.setPower(1);
        } else {
            intake.setPower(0);
        }

        // ....... FEEDER CONTROLS .......
        if (gamepad2.right_bumper) {
            feeder.setPosition(FEEDER_UP);
            loopsOfFeederMotion ++;
        } else {
            feeder.setPosition(FEEDER_DOWN);
            loopsOfFeederMotion = 0;
        }

        // ....... LED LIGHT CODE .......
        indicatorLightCode();





        //telemetry.addData("Label", "Information");

        telemetryM.addData("X Position", robotX);
        telemetryM.addData("Y Position", robotY);
        telemetryM.addData("Heading", Math.toDegrees(headingRadians));
        telemetryM.addData("Relative heading", Math.toDegrees(headingFieldCentric));
        telemetryM.addData("Loops since servo lifted", loopsOfFeederMotion);
        if (Math.abs(flywheelErrorL) < 100) {
            telemetryM.addData("Flywheel1 error", flywheelErrorL);
            telemetryM.addData("Flywheel2 error", flywheelErrorR);
            telemetryM.addData("Error difference", flywheelErrorL - flywheelErrorR);
        }
        getSensedColor();
        telemetryM.update(telemetry);

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


    public void readFromPinpoint() {
        pinpoint.update();
        headingRadians = pinpoint.getHeading(AngleUnit.RADIANS);
        headingFieldCentric = headingRadians - Math.toRadians(driverHeadingDegrees);
        robotX = pinpoint.getPosX(DistanceUnit.INCH);
        robotY = pinpoint.getPosY(DistanceUnit.INCH);
    }


    // ....... Z-TARGETING METHODS .......
    public TargetHeading ZTargetCalculations(double headingOffset) {
        double desiredHeadingDegrees = ratioOfSidesToHeading(goalPos.getX(DistanceUnit.INCH) - robotX,
                                                             goalPos.getY(DistanceUnit.INCH) - robotY);
        telemetryM.addData("Target heading", desiredHeadingDegrees);
        // By default, the intake is the front of the robot. To change that, an offset can be applied here
        return determineRotationDirection(Math.toDegrees(headingRadians) + headingOffset, desiredHeadingDegrees);
    }

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

    // Performs a mod operation but ensures the result will be positive
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
        light2.setPosition(lightColor);

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

    // ....... COLOR SENSOR METHODS .......
    public SensedColors getSensedColor() {

        // Read the direct RGB output of the color sensor
        NormalizedRGBA rawColors = colorSensor.getNormalizedColors();
        telemetryM.addLine("Sensor red: " + rawColors.red + " | normalized: " + (rawColors.red / rawColors.alpha));
        telemetryM.addLine("Sensor green: " + rawColors.green + " | normalized: " + (rawColors.green / rawColors.alpha));
        telemetryM.addLine("Sensor blue: " + rawColors.blue + " | normalized: " + (rawColors.blue / rawColors.alpha));

        // TODO: Set RGB values for purple and green
        // TODO: Return actual detected color
        return SensedColors.UNKNOWN;
    }

} // end class