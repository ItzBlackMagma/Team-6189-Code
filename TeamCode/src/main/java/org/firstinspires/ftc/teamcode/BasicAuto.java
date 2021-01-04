package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class BasicAuto extends LinearOpMode {

    private RobotHardware robot = new RobotHardware(telemetry);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        robot.setLaunchPower(.70);
        robot.move(-1,0,0,.5);
        sleep(400);
        robot.stop();
        sleep(5000);
        robot.setLoadPower(-1);
        sleep(400);
        robot.setLoadPower(0);
        sleep(2000);
        robot.move(-1,0,0,.5);
        sleep(500);
        robot.stop();
        robot.setLaunchPower(.75);
        sleep(5000);
        robot.setLoadPower(-1);
        sleep(1000);
        robot.setLoadPower(0);
        robot.setLaunchPower(0);
        sleep(1000);
        robot.move(0, 1, -.01, .5);
        sleep(2500);
        robot.stop();
    }
}
