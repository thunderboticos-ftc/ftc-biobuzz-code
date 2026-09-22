package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;


public class ControlHub {
    // Initialize Variables
    private final IMU imu;
    
    private double yawAngleRemover;

    public ControlHub(IMU imu) {
        this.imu = imu;

        this.yawAngleRemover = 0;
        
        // Initialize the IMU
        
        IMU.Parameters imuParams = new IMU.Parameters(
            new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, 
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
                
        imu.initialize(imuParams);
        
        // Resets the IMU Values
        
        resetIMUYawAngle();
    }
    
    public static double normalizeAngle(double angle) {
        // Normalizes the value to be < 180 and > -180
        angle = angle % 360.0;
        if(angle > 180) angle -= 360.0;
        if(angle < -180) angle += 360.0;
        
        return angle;
    }
    
    public void resetIMUYawAngle() {
        // Reset the IMU Yaw Angle value
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        yawAngleRemover = angles.getYaw(AngleUnit.DEGREES);
    }
    
    public double getIMUYawAngle() {
        // Get the angle based on the IMU
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        
        double angle = -(angles.getYaw(AngleUnit.DEGREES) - yawAngleRemover);
        angle = normalizeAngle(angle + 360) % 360;
        
        return angle;
    }
    
}
