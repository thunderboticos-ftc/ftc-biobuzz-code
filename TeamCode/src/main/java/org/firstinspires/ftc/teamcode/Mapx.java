package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Mapx {
    private Movement movement;
    private Odometry odometry;
    private ControlHub controlHub;
    private LinearOpMode opMode;
    private double[] initialPos;
    private double[] pos;
    
    public Mapx(Movement movement, Odometry odometry, ControlHub controlHub, double[] initialPos, LinearOpMode opMode) {
        this.movement = movement;
        this.odometry = odometry;
        this.controlHub = controlHub;
        this.initialPos = initialPos;
        this.opMode = opMode;
        this.pos = new double[] {initialPos[0], initialPos[1]};
    }
    
    public void updatePos(double distanceMoved, double angle, Telemetry telemetry) {
        double x = 0;
        double y = distanceMoved;
        double angleAbstract = angle;
        
        if(angleAbstract < 0) {
            angleAbstract *= -1;
        } 
        
        if(angleAbstract > 90) {
            angleAbstract -= 90;
            x = Math.cos(Math.toRadians(angleAbstract)) * distanceMoved;
            y = Math.sin(Math.toRadians(angleAbstract)) * distanceMoved;
            
            y *= -1;
        } else {
            x = Math.sin(Math.toRadians(angleAbstract)) * distanceMoved;
            y = Math.cos(Math.toRadians(angleAbstract)) * distanceMoved;
        }
        
        if(angle < 0) {
            x *= -1;
        }
        
        this.pos = new double[] {x, y};
    }
    
    public double[] getPos() {
        double x = this.pos[0];
        double y = this.pos[1];
        double angle = controlHub.getIMUYawAngle() + initialPos[2];
        angle = controlHub.normalizeAngle(angle + 360) % 360;
        
        return new double[] {x, y, angle};
    }
    
    public void goToAngle(double angle, Telemetry telemetry) {
        double currentAngle = getPos()[2];
        double angleCompensate = angle - currentAngle;
        
        telemetry.addData("c", currentAngle);
        telemetry.addData("a", angle);
        telemetry.addData("angle Compe", angleCompensate);
        telemetry.update();
        
        movement.turnWithCorrection(angleCompensate, 0.5, 0.2);
    }
    
    public double goTo(double[] targetPos, double power) {
        double[] pos = getPos();
        double dx = targetPos[0] - pos[0];
        double dy = targetPos[1] - pos[1];
    
        double distance = Math.sqrt(dx*dx + dy*dy);
    
        // Get the target angle
        double angleTarget = Math.toDegrees(Math.atan2(dx, dy));
    
        // Dot product to decide if it will go backwards
        double angRad = Math.toRadians(pos[2]);
        double forwardX = Math.sin(angRad); // frente no Y
        double forwardY = Math.cos(angRad);
    
        double dotProduct = dx*forwardX + dy*forwardY;
    
        if (dotProduct < 0) {
            angleTarget = (angleTarget + 180) % 360;
        }
    
        // Min difference
        double diff = angleTarget - pos[2];
        diff = (diff + 180) % 360 - 180;
    
        // Turn
        movement.turnWithCorrection(diff, power, 0.2);
    
        // Move
        movement.PIDMax(distance, new double[] {0, dotProduct<0?-1:1}, power);
        
        return dotProduct<0?-distance:distance;
    }
}

