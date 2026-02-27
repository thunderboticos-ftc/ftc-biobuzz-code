package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class Odometry {
    // Initializing pinpoint and constants
    private final GoBildaPinpointDriver pinpoint;
    private final double initialX, initialY;

    public Odometry(GoBildaPinpointDriver pinpoint) {
        // Defining pinpoint and constants
        this.pinpoint = pinpoint;

        initialX = pinpoint.getPosX(DistanceUnit.METER);
        initialY = pinpoint.getPosY(DistanceUnit.METER);
    }
    
    public double getPosX() {
        // Get the odometry x distance
        return  pinpoint.getPosX(DistanceUnit.METER) - initialX;

    }
    
    public double getPosY() {
        // Get the odometry y distance
        return  pinpoint.getPosY(DistanceUnit.METER) - initialY;
    }
}