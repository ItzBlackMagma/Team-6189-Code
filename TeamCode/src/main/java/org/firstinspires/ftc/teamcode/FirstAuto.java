package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous
public class FirstAuto extends LinearOpMode {

    private RobotHardware robot = new RobotHardware(telemetry);
    private Camera camera = new Camera(robot);
    private RingDetector detector = new RingDetector(camera, telemetry);

    private int stackSize = 0, inv = 1, Os = 0, Singles = 0, Quads = 0;
    private double aimIncCount = 100;
    private long aimIncTime = 25;

    private boolean atPoint = false;

    @Override
    public void runOpMode() throws InterruptedException {

        // init phase
        robot.init(hardwareMap);
        camera.activate(hardwareMap);
        detector.initTfod(hardwareMap);
        telemetry.addData("/> INIT", "Activation complete");

        stackSize = detector.detect();
        sleep(500);

        telemetry.addData("/> IsStarted ?", isStarted());
//        while (opModeIsActive() && isStarted()) {
//            camera.track();
//            stackSize = detector.detect();
//            inv = robot.getInvFromTeam(robot.getTeam(camera));
//            telemetry.addData("/> LOCATION", camera.getLocation());
//            telemetry.addData("/> STACK_SIZE", stackSize);
//            telemetry.addData("/> TEAM", robot.getTeam(camera));
//            telemetry.addData("/> INV", inv);
//            telemetry.update();
//        }

        robot.stop();
        waitForStart();
        /*
         *      ******  RUN PHASE ******
         */

        // Scans the starter stack
        for (int i = 0; i < 1000; i++) {
            telemetry.update();
            stackSize = detector.detect();
            telemetry.addData("/> STACK SIZE", stackSize);
            telemetry.addData("/> I", i);

            if(stackSize == 0)
                Os++;
            if(stackSize == 1)
                Singles++;
            if (stackSize == 4)
                Quads++;

            telemetry.addData("/> Zeros", Os);
            telemetry.addData("/> Singles", Singles);
            telemetry.addData("/> Quads", Quads);
        }

        telemetry.addData("/> STACK_SIZE", stackSize);
        telemetry.addData("/> STATUS", "FINISHED");
        telemetry.update();
        sleep(3000);
        // Selects the most likely stack size
//        if(Singles > Os || Quads > Os){
//            if(Quads > Singles){
//                stackSize = 4;
//            } else {
//                stackSize = 1;
//            }
//        } else {
//            stackSize = 0;
//        }

        stackSize = detector.detect();

        telemetry.addData("/> LOCATION", camera.getLocation());
        telemetry.addData("/> STACK_SIZE", stackSize);
        telemetry.addData("/> IMU", robot.getRotation("Z"));
        telemetry.update();
        sleep(3000);

        // moves to the correct wobble goal zone
        switch (stackSize){
            case 0: // Zone A
                robot.move(0, 1, 0, .75);
                sleep(1400);
                robot.move(1, 0, 0, .75);
                sleep(1100);
                robot.move(0,0,1,0.75);
                sleep(300);
                robot.move(0,0,-1,0.75);
                sleep(300);
                robot.stop();
                sleep(100);
                robot.move(-0.2, -1, 0, .75);
                sleep(1200);
                robot.stop();
                break;
            case 1: // Zone B
                robot.move(0, 1, 0, .75);
                sleep(2100);
                robot.move(1, 0, 0, .75);
                sleep(500);
                robot.move(0,-1,0,.5);
                sleep(500);
                robot.move(1, 0, 0, .75);
                sleep(400);
                robot.stop();
                sleep(100);
                robot.move(-0.2, -1, 0, .75);
                sleep(1200);
                robot.stop();
                break;
            case 4: // Zone C
                robot.move(0, 1, 0, .75);
                sleep(2450);
                robot.move(1, 0, 0, .75);
                sleep(1000);
                robot.stop();
                robot.move(0,0,1,0.75);
                sleep(300);
                robot.move(0,0,-1,0.75);
                sleep(300);
                robot.stop();
                sleep(100);
                robot.move(-0.1, -1, 0, .75);
                sleep(2150);
                robot.stop();
                break;
        }

        robot.stop();
        sleep(1000);

        robot.move(-1, 0, 0, .75);
        sleep(1100);
        robot.stop();
        sleep(500);

        // fire rings
        robot.setLaunchPower(.68);
        sleep(2000);
        setLoaderPower(1, 400);
        robot.move(-1,0,0,.5);
        sleep(200);
        robot.stop();
        sleep(2000);
        setLoaderPower(1, 1000);
        robot.setLaunchPower(0);

        // park on launch line
        robot.move(0,1, 0, .75);
        sleep(1500);
        robot.stop();
    }

    public void setLoaderPower(double power, long ms){
        robot.setLoadPower(-power);
        sleep(ms);
        robot.setLoadPower(0);
    }
}
