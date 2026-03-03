package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class Mapx {

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


    public Mapx(HardwareMap hardwareMap) {

        goingTo = false;


        robotHub = RobotHub.getInstance(hardwareMap);
        follower = Constants.createFollower(hardwareMap);

        if(robotMemory.autoFinalPose != null) {
            follower.setPose(robotMemory.autoFinalPose);

        } else {
            follower.setPose(new Pose(72, 72, Math.toRadians(90)));
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
        // Verify if its needed to go
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

        // Gets the target position
        double headingOffset = newHeading;
        double goingToAngle = 0;

        posTargetGoingTo[0] = newPosition[0];
        posTargetGoingTo[1] = newPosition[1];


        angle = AngleUnit.normalizeDegrees(Math.toDegrees(heading) - headingOffset);

        // Updates the relative position
        if(inMeters) {
            relativePos[0] = positionMeters[0];
            relativePos[1] = positionMeters[1];
        } else {
            relativePos[0] = positionInches[0];
            relativePos[1] = positionInches[1];
        }


        // PID values for the y axis
        // P -> 0.1
        // D -> 0.03
        pidValuesY = RobotHub.movement.PIDUpdate(relativePos[1], posTargetGoingTo[1], 0.1, 0.03, lastErrorYGoingTo, lastIYGoingTo);
        correctionY = pidValuesY[0];
        lastErrorYGoingTo = pidValuesY[1];
        lastIYGoingTo = pidValuesY[2];

        // PID values for the X axis
        // P -> 0.2
        // D -> 0.004
        pidValuesX = RobotHub.movement.PIDUpdate(relativePos[0], posTargetGoingTo[0], 0.2, 0.004, lastErrorXGoingTo, lastIXGoingTo);
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

        // heading - 90
        vxRobot = vyField * Math.sin(heading) + vxField * Math.cos(heading);
        vyRobot = vyField * Math.cos(heading) - vxField * Math.sin(heading);

        // Correct the motors
        RobotHub.movement.runMotors(-correctionAngle + vxRobot - vyRobot,
                +correctionAngle + vxRobot + vyRobot,
                -correctionAngle + vxRobot + vyRobot,
                +correctionAngle + vxRobot - vyRobot);
    }

    public double getGoalDistance(int side) {
        // BLUE -> 0
        // RED -> 1

        double[] goalPositionInches;
        double[] goalPositionMeters;

        if(side == 0) {
            goalPositionInches = new double[] {1.75, 142.75};
        } else if(side == 1) {
            goalPositionInches = new double[] {142.25, 142.75};
        } else {
            goalPositionInches = new double[] {0, 0};
        }

        goalPositionMeters = new double[] {goalPositionInches[0] / 39.37, goalPositionInches[1] / 39.37};

        double xRobotMeters = positionMeters[0];
        double yRobotMeters = positionMeters[1];

        double deltaX = Math.abs(goalPositionMeters[0] - xRobotMeters);
        double deltaY = Math.abs(goalPositionMeters[1] - yRobotMeters);

        double distance = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));

        return distance;
    }
}

