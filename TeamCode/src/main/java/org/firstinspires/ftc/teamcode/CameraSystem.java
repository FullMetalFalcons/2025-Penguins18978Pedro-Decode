package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class CameraSystem {

    // Declare the HuskyLens
    HuskyLens lens;

    // Create the different patterns that can be signified by the motif
    public enum Patterns {
        GREEN_PURPLE_PURPLE,
        PURPLE_GREEN_PURPLE,
        PURPLE_PURPLE_GREEN,
        NONE
    }

    public Patterns detectedPattern = Patterns.NONE;
    HuskyLens.Block[] blocks;


    /** Initializes and sets up the HuskyLens
     * @param hardwareMap the hardwareMap from OpMode that should be looked to when initializing the lens */
    public void init(HardwareMap hardwareMap) {

        // Get HuskyLens using hardware map passed in from an OpMode
        lens = hardwareMap.get(HuskyLens.class, "huskylens");

        // Set initial HuskyLens mode
        lens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
    }


    /** Gets the motif pattern signified by the AprilTag that the HuskyLens currently sees */
    public Patterns getHuskyLensPattern() {
        // Get camera data
        blocks = lens.blocks();

        Patterns visiblePattern = Patterns.NONE;

        // Walk through each object seen by the HuskyLens
        for (HuskyLens.Block block : blocks) {
            if (block.id == 1) {
                visiblePattern = Patterns.GREEN_PURPLE_PURPLE;
            } else if (block.id == 2) {
                visiblePattern = Patterns.PURPLE_GREEN_PURPLE;
            } else if (block.id == 3) {
                visiblePattern = Patterns.PURPLE_PURPLE_GREEN;
            }
        }

        return visiblePattern;
    }

    public void logVision(TelemetryManager telemetryM) {
        blocks = lens.blocks();
        for (HuskyLens.Block b : blocks) {
            telemetryM.addData("Block", b.toString());
        }
    }

}
