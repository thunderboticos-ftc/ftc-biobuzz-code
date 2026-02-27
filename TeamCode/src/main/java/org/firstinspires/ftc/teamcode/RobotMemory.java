package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

public final class RobotMemory {

    public static RobotMemory INSTANCE = new RobotMemory();

    // Variables with a default value
    public double[] robotPositionMeters = new double[] {0, 0};
    public double[] robotPositionInches = new double[] {0, 0};
    public double robotAngle = 0;
    public Pose autoFinalPose = null;
    public int allianceSide = 0;




    private RobotMemory() {};
}
