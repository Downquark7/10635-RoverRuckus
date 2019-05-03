package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.robotconfig.initTfod;
import static org.firstinspires.ftc.teamcode.robotconfig.initVuforia;

class asyncVuforiaInit implements Runnable {
    private Thread t;
    private String threadName = "asyncVuforiaInit thread";
    HardwareMap hwMap;

    asyncVuforiaInit(HardwareMap hwMap) {
        System.out.println("Creating " + threadName);
        this.hwMap = hwMap;
    }

    public void run() {
        System.out.println("Running " + threadName);
        initVuforia(hwMap);
        initTfod(hwMap);
        System.out.println("Thread " + threadName + " exiting.");
    }

    public void start() {
        System.out.println("Starting " + threadName);
        if (t == null) {
            t = new Thread(this, threadName);
            t.start();
        }
    }
}
