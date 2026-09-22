package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystem.Mapx;
import org.firstinspires.ftc.teamcode.core.RobotConstants;
import org.firstinspires.ftc.teamcode.hardware.RobotHub;
import org.firstinspires.ftc.teamcode.core.RobotMemory;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class BlueAutonomousCenter extends OpMode {

    private double timeForShoot;


    private RobotHub robotHub;
    private RobotMemory robotMemory;

    private Mapx mapx;

    private int cycle;



    private Follower follower;
    private Timer pathTimer, opModeTimer, waitForShoot, waitForStopAbsorbing;





    private enum PathState {
        START_POS_TO_SHOOT_POS,

    }

    PathState pathState;



    private final Pose startPose = new Pose(135.8504672897196, 71.92523364485983, Math.toRadians(180));
    private final Pose shootPose = new Pose(83.58878504672896, 120.78504672897198, Math.toRadians(90));



    private PathChain startPosToShootPos;

    public void buildPaths() {
        // Put in coordinates for starting pose > ending pose

        startPosToShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
    }








    private void statePathUpdate() {
        switch(pathState) {
            case START_POS_TO_SHOOT_POS:
                // Going to shootPos

                follower.followPath(startPosToShootPos, true);

                if (!follower.isBusy()) {
                    follower.followPath(startPosToShootPos, true);
                }

                break;
        }
    }





    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }



    @Override
    public void init() {
        pathState = PathState.START_POS_TO_SHOOT_POS;
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
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();
        robotMemory.autoFinalPose = follower.getPose();

        RobotHub.limelightCam.valueFilter();

        RobotHub.launcher.setVelocityPIDFOp(RobotConstants.staticLauncherVelocity);
    }
}
