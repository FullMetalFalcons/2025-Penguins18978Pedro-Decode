package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class CompetitionTeleOp extends OpMode {

    // Declare motors, servos, sensors, imus, etc.
    DcMotorEx motorLF, motorRF, motorLB, motorRB, intake, launchL, launchR;
    GoBildaPinpointDriver pinpoint;
    Servo feeder, Light1, Light2;

    // Create constants
    final double TICKS_PER_ROTATION = 28;
    final double RPM_TO_TPS = TICKS_PER_ROTATION / 60;
    /*
        Rotation       Tick       Minute
        --------  *  --------  *  ------
         Minute      Rotation     Second
     */

    final double FEEDER_DOWN = 0.375;
    final double FEEDER_UP = 0.55;

    // Declare and/or initialize other variables
    int velocityRPM = 3000;

    boolean intakeIsActive;
    double headingFieldCentric;
    double headingRadians;
    double robotX;
    double robotY;

    double driverHeadingDegrees;
    Pose2D startingPos;
    Pose2D goalPos;

    // Indicator Light constants
    public final double LED_RED = 0.279;
    public final double LED_BLUE = 0.611;
    public final double LED_GREEN = 0.5;
    public boolean isBlue = true;
    public boolean inPosition = false;


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

        launchR.setDirection(DcMotorSimple.Direction.REVERSE);

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


        // Pinpoint setup
        String pinpointName = Constants.localizerConstants.hardwareMapName;
        GoBildaPinpointDriver.EncoderDirection forwardDirection = Constants.localizerConstants.forwardEncoderDirection;
        GoBildaPinpointDriver.EncoderDirection strafeDirection = Constants.localizerConstants.strafeEncoderDirection;
        GoBildaPinpointDriver.GoBildaOdometryPods resolution = Constants.localizerConstants.encoderResolution;
        double xOffset = Constants.localizerConstants.forwardPodY;  // The forward pod tracks X
        double yOffset = Constants.localizerConstants.strafePodX;   // The strafe pod tracks Y

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, pinpointName);
        pinpoint.setEncoderDirections(forwardDirection, strafeDirection);
        pinpoint.setOffsets(xOffset, yOffset, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(resolution);

    }

    // Runs continuously after INIT is pressed and before START is pressed
    @Override
    public void init_loop() {

        //

    }

    // Runs once when START is pressed
    @Override
    public void start() {

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
        double powerAngular;

        // Modify powers based on robot heading for field-centric drive
        double powerForward = (desiredForward * Math.cos(headingFieldCentric)) - (desiredStrafe * Math.sin(headingFieldCentric));
        double powerStrafe = (desiredStrafe * Math.cos(headingFieldCentric)) + (desiredForward * Math.sin(headingFieldCentric));

        if (gamepad1.a) {
            // Set target heading based on the goal and its position compared to the robot
            towardsGoalHeading = ZTargetCalculations();
            powerAngular = towardsGoalHeading.getDirection() * towardsGoalHeading.getError()/30.0;
        } else {
            powerAngular = -gamepad1.right_stick_x;
        }

        mecanumDriveCode(powerForward, powerStrafe, powerAngular);


        // ....... FLYWHEEL CONTROLS .......
        if (gamepad2.right_trigger > 0.5) {
            // Launch balls
            launchL.setVelocity(velocityRPM * RPM_TO_TPS);
            launchR.setVelocity(velocityRPM * RPM_TO_TPS);
        } else if (gamepad2.left_trigger > 0.5) {
            // Reverse wheels to bring balls back in if necessary
            launchL.setVelocity(-2000 * RPM_TO_TPS);
            launchR.setVelocity(-2000 * RPM_TO_TPS);
        } else {
            launchL.setVelocity(0);
            launchR.setVelocity(0);
        }

        // ....... VELOCITY MODIFICATION .......
        if (gamepad2.dpadUpWasPressed()) {
            velocityRPM += 100;
        } else if (gamepad2.dpadDownWasPressed()) {
            velocityRPM -= 100;
        }
        telemetry.addLine("Wheel Velocity: " + velocityRPM + " RPM");

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
        } else {
            feeder.setPosition(FEEDER_DOWN);
        }

        // ....... COLOR CODE .......
        colors_colors_colors();





        //telemetry.addData("Label", "Information");

        telemetry.addData("X Position", robotX);
        telemetry.addData("Y Position", robotY);
        telemetry.addData("Heading", Math.toDegrees(headingRadians));
        telemetry.addData("Relative heading", Math.toDegrees(headingFieldCentric));
        telemetry.addData("Actual Velocity", launchR.getVelocity() / RPM_TO_TPS);
        telemetry.update();

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

    public void mecanumDriveCode(double forward, double strafe, double angular) {
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
        motorLF.setPower(powerLF /max);
        motorLB.setPower(powerLB /max);
        motorRF.setPower(powerRF /max);
        motorRB.setPower(powerRB /max);
    }


    public void readFromPinpoint() {
        pinpoint.update();
        headingRadians = pinpoint.getHeading(AngleUnit.RADIANS);
        headingFieldCentric = headingRadians - Math.toRadians(driverHeadingDegrees);
        robotX = pinpoint.getPosX(DistanceUnit.INCH);
        robotY = pinpoint.getPosY(DistanceUnit.INCH);
    }


    // ....... Z-TARGETING METHODS .......
    public TargetHeading ZTargetCalculations() {
        double desiredHeadingDegrees = ratioOfSidesToHeading(goalPos.getX(DistanceUnit.INCH) - robotX,
                                                             goalPos.getY(DistanceUnit.INCH) - robotY);
        telemetry.addData("Target heading", desiredHeadingDegrees);
        return determineRotationDirection(Math.toDegrees(headingRadians), desiredHeadingDegrees);
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
        if (Y < 0.0) {
            freeHeading += 180.0;
        } else if (X < 0.0) {
            // Add additional code for quad I to remove all negative values
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

    public void colors_colors_colors() {
        if (!inPosition && gamepad1.backWasPressed()) {
            if (isBlue) {
                isBlue = false;
                Light1.setPosition(LED_RED);
                Light2.setPosition(LED_RED);
            }
            if (!isBlue) {
                isBlue = true;
                Light1.setPosition(LED_BLUE);
                Light2.setPosition(LED_BLUE);

            }
        }

        if (robotX > 32.5 && robotX < 37.5) {
            inPosition = robotY < 40 && robotY > 36;
        }
        else {inPosition = false;}


        if (inPosition) {
            Light1.setPosition(LED_GREEN);
            Light2.setPosition(LED_GREEN);
        }
    }

} // end class