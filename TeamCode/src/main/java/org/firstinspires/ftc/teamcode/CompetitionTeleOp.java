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
    Servo feeder;

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
    int velocityRPM = 3500;

    boolean intakeIsActive;
    double headingFieldCentric;
    double headingRadians;
    double robotX;
    double robotY;

    boolean isRed;
    double driverHeadingDegrees;

    double startingHeadingDegrees;
    Pose2D startingPos;

    public static int number = 0;

    // Runs once when INIT is pressed
    @Override
    public void init() {

        number ++;

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

        if (gamepad1.aWasPressed()) {
            isRed = !isRed;
        }
        driverHeadingDegrees = isRed ? 0 : 180;

        if (gamepad1.bWasPressed()) {
            startingHeadingDegrees = Math.toDegrees(Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x));
            startingHeadingDegrees = modPositive(startingHeadingDegrees, 360);
        }

        telemetry.addData("Robot Heading", startingHeadingDegrees);
        telemetry.addData("Driver Heading", driverHeadingDegrees);
        telemetry.update();
    }

    // Runs once when START is pressed
    @Override
    public void start() {

        startingPos = new Pose2D(DistanceUnit.INCH, 0, 0,
                                 AngleUnit.DEGREES, startingHeadingDegrees);
        pinpoint.setPosition(startingPos);
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

        // Modify powers based on robot heading for field-centric drive
        double powerForward = (desiredForward * Math.cos(headingFieldCentric)) - (desiredStrafe * Math.sin(headingFieldCentric));
        double powerStrafe = (desiredStrafe * Math.cos(headingFieldCentric)) + (desiredForward * Math.sin(headingFieldCentric));

        // Perform vector math to determine the desired powers for each wheel
        double powerLF = powerStrafe + powerForward - powerAngular;
        double powerLB = -powerStrafe + powerForward - powerAngular;
        double powerRF = -powerStrafe + powerForward + powerAngular;
        double powerRB = powerStrafe + powerForward + powerAngular;

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


        // ....... FLYWHEEL CONTROLS .......
        if (gamepad2.left_trigger > 0.5) {
            // Launch balls
            launchL.setVelocity(velocityRPM * RPM_TO_TPS);
            launchR.setVelocity(velocityRPM * RPM_TO_TPS);
        } else if (gamepad2.right_trigger > 0.5) {
            // Reverse wheels to bring balls back in if necessary
            launchL.setVelocity(-600 * RPM_TO_TPS);
            launchR.setVelocity(-600 * RPM_TO_TPS);
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
        } else if (intakeIsActive) {
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





        //telemetry.addData("Label", "Information");

        telemetry.addData("X Position", robotX);
        telemetry.addData("Y Position", robotY);
        telemetry.addData("Heading", Math.toDegrees(headingRadians));
        telemetry.addData("Relative heading", Math.toDegrees(headingFieldCentric));
        telemetry.addData("Static Number", number);
        telemetry.addData("Actual Velocity", launchR.getVelocity());
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

    public void readFromPinpoint() {
        pinpoint.update();
        headingRadians = pinpoint.getHeading(AngleUnit.RADIANS);
        headingFieldCentric = headingRadians - Math.toRadians(driverHeadingDegrees);
        robotX = pinpoint.getPosX(DistanceUnit.INCH);
        robotY = pinpoint.getPosY(DistanceUnit.INCH);
    }

    // Performs a mod operation but ensures the result will be positive
    public double modPositive(double number, double divisor) {
        return ((number % divisor) + divisor) % divisor;
    }

} // end class