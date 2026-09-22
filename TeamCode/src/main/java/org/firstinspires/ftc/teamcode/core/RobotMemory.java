package org.firstinspires.ftc.teamcode.core;

import com.pedropathing.geometry.Pose;

public final class RobotMemory {

    public static RobotMemory INSTANCE = new RobotMemory();

    // Variables with a default value
    public Pose autoFinalPose = null;
    public int allianceSide = 0;


    private RobotMemory() {}
}
