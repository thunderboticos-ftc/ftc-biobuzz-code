package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystem.Cam;
import org.firstinspires.ftc.teamcode.subsystem.Launcher;
import org.firstinspires.ftc.teamcode.subsystem.Movement;
import org.firstinspires.ftc.teamcode.subsystem.Odometry;


public class RobotHub {
    // Variable initialization

    // Hardware
    public static RobotHub instance;
    
    public IMU imu;
    public DcMotorEx motorLeftFront;
    public DcMotorEx motorRightFront;
    public DcMotorEx motorLeftBack;
    public DcMotorEx motorRightBack;
    public DcMotorEx launcherMotor;
    public DcMotor intakeMotor;
    public DcMotor midtakeMotor;


    public Servo leftServo;
    public Servo rightServo;

    public  Servo testeServo;
    public Limelight3A limelight;
    public DcMotor led;
    public static GoBildaPinpointDriver pinpoint;

    // Classes
    public static Cam limelightCam;
    public static Movement movement;
    public static ControlHub controlHub;
    public Odometry odometry;
    public static Launcher launcher;

    // Constants
    private final double X_POD_OFFSET_MM, Y_POD_OFFSET_MM;

    public RobotHub(HardwareMap hardwareMap) {
        // Pinpoint settings
        X_POD_OFFSET_MM = 142;
        Y_POD_OFFSET_MM = 90.427;

        // pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // pinpoint.setOffsets(X_POD_OFFSET_MM, Y_POD_OFFSET_MM, DistanceUnit.MM);

        // pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);

        // pinpoint.setEncoderDirections(
        //         GoBildaPinpointDriver.EncoderDirection.FORWARD,
        //         GoBildaPinpointDriver.EncoderDirection.FORWARD
        // );

        // pinpoint.resetPosAndIMU();

        // Defining motors and sensors
        imu = hardwareMap.get(IMU.class, "imu");
        
        motorLeftFront = hardwareMap.get(DcMotorEx.class, "motorLeftFront");
        motorRightFront = hardwareMap.get(DcMotorEx.class, "motorRightFront");
        motorLeftBack = hardwareMap.get(DcMotorEx.class, "motorLeftBack");
        motorRightBack = hardwareMap.get(DcMotorEx.class, "motorRightBack");

        // launcherMotor = hardwareMap.get(DcMotorEx.class, "launcher");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        // midtakeMotor = hardwareMap.get(DcMotor.class, "midtakeMotor");

        // leftServo = hardwareMap.get(Servo.class, "leftServo");
        // rightServo = hardwareMap.get(Servo.class, "rightServo");
        testeServo = hardwareMap.get(Servo.class, "testeServo");

        // led = hardwareMap.get(DcMotor.class, "led");


        // limelight = hardwareMap.get(Limelight3A.class, "limelight");


        // Initializing util classes
        // limelightCam = new Cam(limelight);
        // controlHub = new ControlHub(imu);
        
        // odometry = new Odometry(pinpoint);
        // movement = new Movement(motorLeftFront, motorRightFront, motorLeftBack, motorRightBack, odometry, controlHub, pinpoint);

        launcher = new Launcher(launcherMotor);

        instance = this;
    }

    public static RobotHub getInstance(HardwareMap hardwareMap) {
        // getInstance
        if(instance == null) {
            return new RobotHub(hardwareMap);
        } else {
            return instance;
        }
    }

    public static void reset() {
        // Reset the instance

        instance = null;
    }
}
