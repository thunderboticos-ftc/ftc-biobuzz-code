package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.control.GamepadController;
import org.firstinspires.ftc.teamcode.core.RobotMemory;
import org.firstinspires.ftc.teamcode.core.TelemetryData;
import org.firstinspires.ftc.teamcode.hardware.RobotHub;
import org.firstinspires.ftc.teamcode.subsystem.Mapx;

@TeleOp

public class Main extends LinearOpMode {
    Mapx mapx;
    
    double[] initialPos;
    double initialAngle;

    int allianceSide;
    
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

        // mapx = new Mapx(hardwareMap);
        // robotMemory = RobotMemory.INSTANCE;
        // allianceSide = robotMemory.allianceSide;
        // robotMemory.allianceSide = 0;
        // robotMemory.autoFinalPose = null;

        // gamepadController = new GamepadController(gamepad1, gamepad2, RobotHub.movement, RobotHub.limelightCam);
        // telemetryData = new TelemetryData(telemetry, RobotHub.limelightCam);
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();  
        
        // run until the end of the match (driver presses STOP)
        while(opModeIsActive()) {

            robotHub.testeServo.setPosition(1);



            // Get the x and y axis
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            double power = 0.8;

            // Get the needed power for each motor using the direction vector
            double leftFrontPower = (y + x + turn) * power;
            double rightFrontPower = (y - x - turn) * power;
            double leftBackPower = (y - x + turn) * power;
            double rightBackPower = (y + x - turn) * power;

            // Normalize to be between -1 and 1
            double max = Math.max(1.0,
                    Math.max(Math.abs(leftFrontPower),
                            Math.max(Math.abs(rightFrontPower),
                                    Math.max(Math.abs(leftBackPower),
                                            Math.abs(rightBackPower)))));

            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;

            robotHub.motorLeftFront.setPower(leftFrontPower);
            robotHub.motorRightFront.setPower(rightFrontPower);
            robotHub.motorLeftBack.setPower(leftBackPower);
            robotHub.motorRightBack.setPower(rightBackPower);

            if(gamepad1.right_trigger > 0) {

                robotHub.intakeMotor.setPower(0.8);

            } else {
                robotHub.intakeMotor.setPower(0);
            }






            // robotHub.testeMotor.setPower(0.8);

            // gamepadController.moveUsingGamepad();
            // gamepadController.keyStatesUpdate();
            // gamepadController.changeVelocityUsingGamepad();
            // gamepadController.lockPosition();
            // gamepadController.rumbleOnDangerousAreas(allianceSide, mapx);

            // RobotHub.limelightCam.valueFilter();

            // RobotHub.pinpoint.update();

            // mapx.update();


            // telemetry.addData("Alliance", allianceSide);

            // telemetry.update();
        }
    }
}
