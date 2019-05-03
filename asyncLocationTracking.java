package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.drive.Localizer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.robotconfig.initTfod;
import static org.firstinspires.ftc.teamcode.robotconfig.initVuforia;

class asyncLocationTracking implements Runnable {
    private Thread t;
    private String threadName = "asyncLocationTracking thread";
    HardwareMap hwMap;
    Localizer locale;
    boolean running = false;

    asyncLocationTracking(HardwareMap hwMap, Localizer locale) {
        System.out.println("Creating " + threadName);
        this.hwMap = hwMap;
        this.locale = locale;
    }

    public void run() {
        System.out.println("Running " + threadName);
        while(running) {

        }
        System.out.println("Thread " + threadName + " exiting.");
    }

    public void start() {
        System.out.println("Starting " + threadName);
        running = true;
        if (t == null) {
            t = new Thread(this, threadName);
            t.start();
        }
    }

    public void stop() {
        running = false;
    }
}
