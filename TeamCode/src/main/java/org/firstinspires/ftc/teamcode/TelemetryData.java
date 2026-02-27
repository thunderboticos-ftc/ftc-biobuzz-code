package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TelemetryData {
    private final Telemetry telemetry;
    private final Launcher launcher;
    private final Cam cam;
    
    public TelemetryData(Telemetry telemetry, Launcher launcher, Cam cam) {
        this.telemetry = telemetry;
        this.launcher = launcher;
        this.cam = cam;
        
        telemetry.setMsTransmissionInterval(11);
    }
    
    public void setLauncherStateData() {
        // Get the launcher real needed velocity
        double targetDistance = cam.getAprilTagDistance() / 100;
        double neededVelocity = launcher.getLauncherNeededAngleAndVelocity(targetDistance)[0];            
        double realVelocity = launcher.getCompensateLauncherVelocity(neededVelocity);
        
        // Set the telemetry state according to the currently velocity
        telemetry.addData("Launcher State:", launcher.isLauncherReadToGo(realVelocity, 0.1)?"Ready":"Wait");
    }
    
    public void setCurrentLauncherVelocityData() {
        double velocity = launcher.getVelocity();
        
        telemetry.addData("Launcher Velocity:", velocity);
    }
}
