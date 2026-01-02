package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class CompetitionTeleOp extends OpMode {

    //Initialize motors, servos, sensors, imus, etc.
    DcMotorEx motorLF, motorRF, motorLB, motorRB, intake, launchL, launchR;
    int velocity;


    // Runs once when INIT is pressed
    @Override
    public void init() {

        // Setup drive motors based on constants file
        motorLF = (DcMotorEx) hardwareMap.dcMotor.get(Constants.driveConstants.leftFrontMotorName);
        motorLB = (DcMotorEx) hardwareMap.dcMotor.get(Constants.driveConstants.leftRearMotorName);
        motorRF = (DcMotorEx) hardwareMap.dcMotor.get(Constants.driveConstants.rightFrontMotorName);
        motorRB = (DcMotorEx) hardwareMap.dcMotor.get(Constants.driveConstants.rightRearMotorName);

        // Setup other motors
        intake = (DcMotorEx) hardwareMap.dcMotor.get("intake");
        launchL = (DcMotorEx) hardwareMap.dcMotor.get("launchL");
        launchR = (DcMotorEx) hardwareMap.dcMotor.get("launchR");

        launchR.setDirection(DcMotorSimple.Direction.REVERSE);

        // Use the following line as a template for defining new servos
        //Claw = (Servo) hardwareMap.servo.get("claw");

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

    }

    // Runs continually after INIT is pressed and before STOP is pressed
    @Override
    public void loop() {

        // Mecanum drive code
        double powerX = 0.0;  // Desired power for strafing           (-1 to 1)
        double powerY = 0.0;  // Desired power for forward/backward   (-1 to 1)
        double powerAng = 0.0;  // Desired power for turning          (-1 to 1)

        // Set the desired powers based on joystick inputs (-1 to 1)
        powerX = gamepad1.left_stick_x;
        powerY = -gamepad1.left_stick_y;
        powerAng = -gamepad1.right_stick_x;

        // Perform vector math to determine the desired powers for each wheel
        double powerLF = powerX + powerY - powerAng;
        double powerLB = -powerX + powerY - powerAng;
        double powerRF = -powerX + powerY + powerAng;
        double powerRB = powerX + powerY + powerAng;

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


        launchL.setVelocity( gamepad2.left_trigger > 0.5 ?    velocity : 0 );
        launchR.setVelocity( gamepad2.right_trigger > 0.5 ?   velocity : 0 );

        if (gamepad2.dpadUpWasPressed()) {
            velocity += 100;
        } else if (gamepad2.dpadDownWasPressed()) {
            velocity -= 100;
        }
        telemetry.addData("Wheel Velocity", velocity);

        if (gamepad2.left_bumper) {
            intake.setPower(1);
        } else {
            intake.setPower(0);
        }





        //telemetry.addData("Label", "Information");
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

} // end class