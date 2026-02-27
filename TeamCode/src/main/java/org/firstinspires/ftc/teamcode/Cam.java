package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Cam {
    private final Limelight3A cam;

    private final double launcherDistanceFromCam;


    // Filter Variables

    boolean filterInit;
    double valueFiltered;
    ElapsedTime filterTimer;
    double tau;


    public double goldenWeightedAverage(double lighter, double heavier) {
        double phi = (1 + Math.sqrt(5)) / 2;

        double wHeavier = 1 / phi;
        double wLighter = 1 - wHeavier;


        return wLighter * lighter + wHeavier * heavier;
    }

    public void valueFilter() {
        // Resets the filter with a velocityValue
        if(!this.filterInit) {
            valueFiltered = getAprilTagDistance();
            this.filterInit = true;
            return;
        }

        // Gets the delta time
        double dt = filterTimer.time();
        filterTimer.reset();

        // Gets alpha based on the natural exponential for a
        double alpha = 1 - Math.exp(-(dt / this.tau));
        this.valueFiltered = valueFiltered + alpha * (getAprilTagDistance() - valueFiltered);
    }

    public Cam(Limelight3A cam) {
        this.cam = cam;
        this.launcherDistanceFromCam = 27;
        cam.pipelineSwitch(0);
        cam.start();
    }
    
    public double getAprilTagDistance() {
        double ta = getAprilTagResults()[2];
        double distanceCalibrator = 175.9266;
        
        // Confirm if the april tag is in the camera FOV
        if(ta == -1)
            return -1;
        
        // Get the distance by using april tag area on the screen and the calibrator
        // d = k * (1 / sqrt(ta))

        // Variables being set
        // A constant for calibrate the distance
        return (distanceCalibrator * (1 / Math.sqrt(ta))) + launcherDistanceFromCam;
    }
    
    public double[] getAprilTagResults() {
        LLResult results = cam.getLatestResult();
        double tx = results.getTx();
        double ty = results.getTy();
        double ta = results.getTa();
        
        if(!results.isValid())
            return new double[] {-1, -1, -1};
            
        return new double[] {tx, ty, ta};
    }
}
