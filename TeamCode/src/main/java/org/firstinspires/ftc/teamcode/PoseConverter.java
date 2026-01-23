package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import com.pedropathing.geometry.Pose;

public class PoseConverter {

    /** Convert a PedroPathing Pose into an FTC Pose2D */
    public static Pose2D poseToPose2D(Pose pedroPose) {
        return new Pose2D(DistanceUnit.INCH, pedroPose.getX(), pedroPose.getY(),
                          AngleUnit.RADIANS, pedroPose.getHeading());
    }

    /** Convert an FTC Pose2D into a PedroPathing Pose */
    public static Pose pose2DToPose(Pose2D ftcPose) {
        return new Pose(ftcPose.getX(DistanceUnit.INCH),
                        ftcPose.getY(DistanceUnit.INCH),
                        ftcPose.getHeading(AngleUnit.RADIANS));
    }

}
