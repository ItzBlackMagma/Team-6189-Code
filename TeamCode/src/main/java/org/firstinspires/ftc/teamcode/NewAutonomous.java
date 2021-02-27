package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class NewAutonomous extends LinearOpMode {
    Robot robot = new Robot(telemetry);
    final double DEFAULT_SPEED = 0.8;
    int stackSize = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.resetEncoders();
        robot.toPosition();
        waitForStart();

        stackSize = robot.camera.stackSize();
        telemetry.addData("/> STACK SIZE", stackSize);
        telemetry.update();
        sleep(100);

        switch (stackSize){  // move the first wobble goal
            case 0:
                moveToPoint(Locations.WOBBLE_ZONE_A - 18, 4000);
                moveToPoint(-Locations.WOBBLE_ZONE_A + 18, 4000);
                break;
            case 1:
                moveToPoint(Locations.WOBBLE_ZONE_B - 18, 6000);
                moveToPoint(-Locations.WOBBLE_ZONE_B + 18, 6000);
                break;
            case 4:
                moveToPoint( Locations.WOBBLE_ZONE_C - 18, 7000);
                moveToPoint(-Locations.WOBBLE_ZONE_C + 18, 7000);
                break;
        }
        moveToPoint(0, Locations.LINE_2 * .75, .08, 0, 2000); // navigate to the second wobble goal

        switch (stackSize){ // move the second wobble goal
            case 0:
                moveToPoint(Locations.WOBBLE_ZONE_A - 18, 4000);
                moveToPoint(-Locations.WOBBLE_ZONE_A + 18, 4000);
                break;
            case 1:
                moveToPoint(Locations.WOBBLE_ZONE_B - 18, 6000);
                moveToPoint(-Locations.WOBBLE_ZONE_B + 18, 6000);
                break;
            case 4:
                moveToPoint( Locations.WOBBLE_ZONE_C - 18, 7000);
                moveToPoint(-Locations.WOBBLE_ZONE_C + 18, 7000);
                break;
        }

    }

    void moveToPoint(double pos, double offset, double xPower, double yPower, long sleep){
        robot.move(xPower, yPower,0,1);
        robot.setPos(pos - offset,pos + offset,pos - offset,pos + offset);
        robot.toPosition();
        sleep(sleep);
        robot.resetEncoders();
    }

    void moveToPoint(double pos, long sleep){
        moveToPoint(pos, 0, 0, DEFAULT_SPEED, sleep);
    }

    void moveToPoint(double pos, double power, long sleep){
        moveToPoint(pos, 0, 0, power, sleep);
    }
}
