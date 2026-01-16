package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class DriveOffAuto extends LinearOpMode {

    // Declare motors, servos, sensors, imus, etc.
    DcMotorEx motorLF, motorRF, motorLB, motorRB;


    // Runs once when INIT is pressed
    @Override
    public void runOpMode() {

        // Setup drive motors based on constants file
        motorLF = hardwareMap.get(DcMotorEx.class, Constants.driveConstants.leftFrontMotorName);
        motorLB = hardwareMap.get(DcMotorEx.class, Constants.driveConstants.leftRearMotorName);
        motorRF = hardwareMap.get(DcMotorEx.class, Constants.driveConstants.rightFrontMotorName);
        motorRB = hardwareMap.get(DcMotorEx.class, Constants.driveConstants.rightRearMotorName);

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


        waitForStart();

        if (opModeIsActive()) {

            // Start strafing to the right if the robot is blue or left if the robot is red
            if (AALocationChooser.chosenColor == AALocationChooser.AllianceColor.BLUE) {
                strafeWithPower(-1);
            } else {
                strafeWithPower(1);
            }

            // Wait to let the robot run
            sleep(500);

            // Stop the robot
            strafeWithPower(0);

        }

    }

    public void strafeWithPower(double power) {
        motorLF.setPower(power);
        motorLB.setPower(-power);
        motorRF.setPower(-power);
        motorRB.setPower(power);
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