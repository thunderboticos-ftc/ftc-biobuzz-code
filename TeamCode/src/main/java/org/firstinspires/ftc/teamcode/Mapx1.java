package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class Mapx1 {

    RobotMemory robotMemory = RobotMemory.INSTANCE;

    Follower follower;

    double[] positionMeters;
    double[] positionInches;
    double heading;

    RobotHub robotHub;


    // goToGoal

    double lastErrorYGoingTo;
    double lastIYGoingTo;
    double lastErrorXGoingTo;
    double lastIXGoingTo;
    double lastErrorAngleGoingTo;
    double lastIAngleGoingTo;


    boolean goingTo;


    public Mapx1(HardwareMap hardwareMap) {


        goingTo = false;


        robotHub = RobotHub.getInstance(hardwareMap);
        follower = Constants.createFollower(hardwareMap);

        if(robotMemory.autoFinalPose != null) {
            follower.setPose(robotMemory.autoFinalPose);

        } else {
            follower.setPose(new Pose(72, 71, 90));
        }

        this.positionInches = new double[] {follower.getPose().getX(), follower.getPose().getY()};
        this.positionMeters = new double[] {follower.getPose().getX() / 39.37, follower.getPose().getY() / 39.37}; // Converts to meters
        this.heading = follower.getHeading();
    }

    public void update() {
        follower.update();

        Pose pose = follower.getPose(); // Inches

        this.positionInches = new double[] {pose.getX(), pose.getY()};
        this.positionMeters = new double[] {pose.getX() / 39.37, pose.getY() / 39.37}; // Converts to meters

        this.heading = pose.getHeading();
    }

    public void goToGoal(double[] newPosition, double newHeading, boolean inMeters, boolean started) {
        // Verify if its needed to lock in
        if(!started) {
            goingTo = false;
            return;
        }

        double heading = this.heading;

        // Base variable values
        double[] pidValuesY;
        double[] pidValuesX;
        double[] pidValuesAngle;

        double correctionY;
        double correctionX;
        double correctionAngle;

        double angle = 0;

        double[] relativePos = new double[2];

        double[] posTargetGoingTo = new double[2];

        // Gets the first position when locked
        double headingOffset = newHeading;
        double goingToAngle = 0;

        posTargetGoingTo[0] = newPosition[0];
        posTargetGoingTo[1] = newPosition[1];


        angle = AngleUnit.normalizeDegrees(heading - headingOffset);

        // Updates the relative position
        if(inMeters) {
            relativePos[0] = positionMeters[0];
            relativePos[1] = positionMeters[1];
        } else {
            relativePos[0] = positionInches[0];
            relativePos[1] = positionInches[1];
        }


        // PID values for the y axis
        // P -> 1.3
        // D -> 0.7
        pidValuesY = RobotHub.movement.PIDUpdate(relativePos[1], posTargetGoingTo[1], 1.3, 0.7, lastErrorYGoingTo, lastIYGoingTo);
        correctionY = pidValuesY[0];
        lastErrorYGoingTo = pidValuesY[1];
        lastIYGoingTo = pidValuesY[2];


        // PID values for the X axis
        // P -> 1.5
        // D -> 0.1
        pidValuesX = RobotHub.movement.PIDUpdate(relativePos[0], posTargetGoingTo[0], 1.5, 0.1, lastErrorXGoingTo, lastIXGoingTo);
        correctionX = pidValuesX[0];
        lastErrorXGoingTo = pidValuesX[1];
        lastIXGoingTo = pidValuesY[2];


        // PID values for the angle
        // P -> 0.02
        // D -> 0.0005
        pidValuesAngle = RobotHub.movement.PIDUpdate(angle, goingToAngle, 0.02, 0.0005, lastErrorAngleGoingTo, lastIAngleGoingTo);
        correctionAngle = pidValuesAngle[0];
        lastErrorAngleGoingTo = pidValuesAngle[1];
        lastIAngleGoingTo = pidValuesY[2];

        double vxField, vyField, vxRobot, vyRobot;

        vxField = correctionX;
        vyField = correctionY;

        vxRobot = vyField * Math.sin(Math.toRadians(-heading)) + vxField * Math.cos(Math.toRadians(-heading));
        vyRobot = vyField * Math.cos(Math.toRadians(-heading)) - vxField * Math.sin(Math.toRadians(-heading));


        // Correct the motors
        RobotHub.movement.runMotors(correctionAngle + vxRobot - vyRobot,
                -correctionAngle + vxRobot + vyRobot,
                correctionAngle + vxRobot + vyRobot,
                -correctionAngle + vxRobot - vyRobot);

    }
}

