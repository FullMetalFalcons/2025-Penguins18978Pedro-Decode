package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable
public class LaunchSystem {

    // Declare motors, servos, and sensors
    DcMotorEx intake, launchL, launchR;
    Servo feeder;

    // State machine variables
    ElapsedTime stateTimer = new ElapsedTime();
    LauncherState currentState = LauncherState.IDLE;
    int ballsToFire = 0;

    public enum LauncherState {
        IDLE,
        LOAD,
        PREPARE,
        LAUNCH,
        FOLLOW_THROUGH
    }

    // Static constants that can be tuned in Panels
    public static double FEEDER_DOWN = 0.37;
    public static double FEEDER_UP = 0.47;

    public static double FLYWHEEL_F = 13.0;
    public static double FLYWHEEL_P = 200.0;


    // Unit conversion constants
    final double TICKS_PER_ROTATION = 28;
    final double TPS_PER_RPM = TICKS_PER_ROTATION / 60;
    /*
        Rotation       Tick       Minute
        --------  *  --------  *  ------
         Minute      Rotation     Second
     */
    int velocityRpm = 3200;


    /** Initializes motors, sets motor directions, and sets up PIDF constants for launch-related motors
     * @param hardwareMap the hardwareMap from OpMode that should be looked to when initializing motors */
    public void init(HardwareMap hardwareMap) {

        // Get motors using hardware map passed in from an OpMode
        intake = (DcMotorEx) hardwareMap.dcMotor.get("intake");
        launchL = (DcMotorEx) hardwareMap.dcMotor.get("launchL");
        launchR = (DcMotorEx) hardwareMap.dcMotor.get("launchR");
        feeder = hardwareMap.servo.get("feeder");

        // Set motor properties
        launchR.setDirection(DcMotorSimple.Direction.REVERSE);
        launchL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launchR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set flywheel Feedforward and Proportional constants
        launchL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(FLYWHEEL_P,0,0, FLYWHEEL_F));
        launchR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(FLYWHEEL_P,0,0, FLYWHEEL_F));


        // Set initial positions and powers
        feeder.setPosition(FEEDER_DOWN);
    }

    /** Runs the state machine logic and any motors that need to be run as part of this state */
    public void update() {

        // State machine for autonomous
        switch (currentState) {
            case IDLE:
                // If balls are queued to be launched, then begin the launch process
                if (ballsToFire > 0) {
                    setState(LauncherState.LOAD);
                }
                break;
            case LOAD:
                // Intake the next ball and begin spinning up the flywheels
                setLauncherVelocity(velocityRpm);
                intake.setPower(1);
                if (stateTimer.seconds() > 0.3) {
                    // Stop the intake and switch to the next state
                    setState(LauncherState.PREPARE);
                    intake.setPower(0);
                }
                break;
            case PREPARE:
                // Wait for the flywheels to finish spinning up
                // Also wait a mandatory pause to allow the ball to settle
                if (((getFlywheelError(launchL) < 80 && getFlywheelError(launchR) < 80) || stateTimer.seconds() > 2.0)
                                                                                        && stateTimer.seconds() > 0.5) {
                    setState(LauncherState.LAUNCH);
                }
                break;
            case LAUNCH:
                // Feed a ball into the flywheels and wait for the servo to finish moving
                feeder.setPosition(FEEDER_UP);
                if (stateTimer.seconds() > 0.5) {
                    // Mark that a ball has been fired and reset the feeder
                    ballsToFire --;
                    feeder.setPosition(FEEDER_DOWN);

                    // If more balls need to be launched, load in the next ball
                    // If no more balls are queued up, then end the sequence
                    if (ballsToFire > 0) {
                        setState(LauncherState.LOAD);
                    } else {
                        setState(LauncherState.FOLLOW_THROUGH);
                    }
                }
                break;
            case FOLLOW_THROUGH:
                // Keep the flywheels spinning for a little longer to ensure that the last ball fires properly
                if (stateTimer.seconds() > 0.5) {
                    setLauncherVelocity(0);
                    setState(LauncherState.IDLE);
                }
        }
    }


    // ....... STATE MACHINE METHODS .......
    public void setState(LauncherState newState) {
        stateTimer.reset();
        currentState = newState;
    }

    /** Interrupt any state machine actions that are currently in progress */
    public void stopActions() {
        if (isBusy()) {
            // Reset all motors to their default positions/powers
            intake.setPower(0);
            setLauncherVelocity(0);
            feeder.setPosition(FEEDER_DOWN);

            // Clear any queued balls
            ballsToFire = 0;

            // Exit the state machine and return to idle
            setState(LauncherState.IDLE);
        }
    }

    /** Tell the state machine to load and launch a certain number of balls */
    public void fireBalls(int numBalls) {
        if (!isBusy()) {  ballsToFire = numBalls;  }
    }
    public boolean isBusy() {
        return (currentState != LauncherState.IDLE) || (ballsToFire > 0);
    }

    // ....... MOTOR CONTROL METHODS .......
    /** Set the position of the feeder servo */
    public void setFeederPosition(double position) {
        feeder.setPosition(position);
    }
    /** Set the power of the intake motor */
    public void setIntakePower(double power) {
        intake.setPower(power);
    }
    /** Set the velocity of both launcher flywheels
     * @param rpm the desired velocity in rotations per minute
     */
    public void setLauncherVelocity(int rpm) {
        launchL.setVelocity(rpm * TPS_PER_RPM);
        launchR.setVelocity(rpm * TPS_PER_RPM);
    }

    // ....... FLYWHEEL VELOCITY METHODS .......
    /** Get the error between a flywheel's target velocity and its actual velocity
     * @param flywheel the DcMotorEx that will have its error checked
     * @return the difference between target and actual velocity in rotations per minute
     */
    public double getFlywheelError(DcMotorEx flywheel) {
        return velocityRpm - (flywheel.getVelocity() / TPS_PER_RPM);
    }
    /** Get the error between a flywheel's target velocity and its actual velocity
     * @param flywheel the DcMotorEx that will have its error checked
     * @param targetRpm the target velocity of the flywheel in rotations per minute
     * @return the difference between target and actual velocity in rotations per minute
     */
    public double getFlywheelError(DcMotorEx flywheel, int targetRpm) {
        return targetRpm - (flywheel.getVelocity() / TPS_PER_RPM);
    }

    /** Set the target launch velocity in rotations per minute. Does not activate any motors: Only stores the value for future use */
    public void setTargetVelocity(int velocity) {
        velocityRpm = velocity;
    }
    /** Modify the target launch velocity in rotations per minute. Does not activate any motors: Only stores the value for future use */
    public void changeTargetVelocity(int addition) {
        velocityRpm += addition;
    }

}