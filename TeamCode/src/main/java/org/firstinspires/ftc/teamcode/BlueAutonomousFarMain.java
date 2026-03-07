package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class BlueAutonomousFarMain extends OpMode {

    private double timeForShoot;


    private RobotHub robotHub;
    private RobotMemory robotMemory;

    private Mapx mapx;

    private int cycle;



    private Follower follower;
    private Timer pathTimer, opModeTimer, waitForShoot, waitForStopAbsorbing;





    private enum PathState {
        START_POS_TO_FINAL_POS,
        STOPED

    }

    PathState pathState;



    private final Pose startPose = new Pose(56.25, 7, Math.toRadians(90));
    private final Pose finalPos = new Pose(39, 8, Math.toRadians(90));




    private PathChain startPosToFinalPos;


    public void buildPaths() {
        // Put in coordinates for starting pose > ending pose

        startPosToFinalPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, finalPos))
                .setLinearHeadingInterpolation(startPose.getHeading(), finalPos.getHeading())
                .build();
    }








    private void statePathUpdate() {
        switch(pathState) {
            case START_POS_TO_FINAL_POS:
                // Going to shootPos

                follower.followPath(startPosToFinalPos, false);
                setPathState(pathState.STOPED);

                break;

            case STOPED:
                break;
        }
    }





    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }



    @Override
    public void init() {
        pathState = PathState.START_POS_TO_FINAL_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        waitForShoot = new Timer();
        waitForStopAbsorbing = new Timer();

        timeForShoot = 0;


        cycle = 0;


        opModeTimer.resetTimer();


        RobotHub.reset();
        robotHub = RobotHub.getInstance(hardwareMap);
        robotMemory = RobotMemory.INSTANCE;

        follower = Constants.createFollower(hardwareMap);

        mapx = new Mapx(hardwareMap);

        buildPaths();
        follower.setPose(startPose);
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
        robotMemory.allianceSide = 0;
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();
        robotMemory.autoFinalPose = follower.getPose();
        mapx.update();
    }
}
