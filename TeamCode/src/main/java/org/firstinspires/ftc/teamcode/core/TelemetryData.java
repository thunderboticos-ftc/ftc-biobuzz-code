package org.firstinspires.ftc.teamcode.core;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.Cam;

public class TelemetryData {
    private final Telemetry telemetry;
    private final Cam cam;
    
    public TelemetryData(Telemetry telemetry, Cam cam) {
        this.telemetry = telemetry;
        this.cam = cam;
        
        telemetry.setMsTransmissionInterval(11);
    }
}
