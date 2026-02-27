package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp

public class Main extends LinearOpMode {
    Mapx1 mapx;
    
    double[] initialPos;
    double initialAngle;
    
    RobotHub robotHub;
    RobotMemory robotMemory;

    GamepadController gamepadController;
    TelemetryData telemetryData;


    @Override   
    public void runOpMode() {

        initialPos = new double[] {0, 0, 0};
        initialAngle = 0;

        RobotHub.reset();
        robotHub = RobotHub.getInstance(hardwareMap);

        mapx = new Mapx1(hardwareMap);
        robotMemory = RobotMemory.INSTANCE;
        robotMemory.autoFinalPose = null;

        gamepadController = new GamepadController(gamepad1, gamepad2, RobotHub.movement, RobotHub.launcher, RobotHub.limelightCam);
        telemetryData = new TelemetryData(telemetry, RobotHub.launcher, RobotHub.limelightCam);
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();  
        
        // run until the end of the match (driver presses STOP)
        while(opModeIsActive()) {
            gamepadController.moveUsingGamepad();
            gamepadController.launchUsingGamepad();
            //gamepadController.absorbUsingGamepad();
            gamepadController.transportBallToLauncher();
            gamepadController.keyStatesUpdate();
            gamepadController.correctAngleForTargetUsingGamepad();
            gamepadController.launchUsingGamepadWithoutCam();
            gamepadController.ejectArtefacts();
            gamepadController.changeVelocityUsingGamepad();
            gamepadController.lockPosition();
            gamepadController.changeLauncherVelocityUsingGamepad();
            gamepadController.transportToShooter();
            gamepadController.rumbleOnDangerousAreas(robotMemory.allianceSide, mapx);
            gamepadController.goToEndLocal(robotMemory.allianceSide, mapx);


            RobotHub.launcher.updateLauncherVelocityAndRps();
            RobotHub.launcher.verifyLaunches();
            RobotHub.launcher.valueFilter();

            RobotHub.pinpoint.update();

            mapx.update();


            telemetry.addData("XM", mapx.positionMeters[0]);
            telemetry.addData("YM", mapx.positionMeters[1]);
            telemetry.addData("XI", mapx.positionInches[0]);
            telemetry.addData("YI", mapx.positionInches[1]);
            telemetry.addData("H", mapx.heading);
            telemetry.update();
























        }
    }
}
