package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class RobotHardware {

    Telemetry telemetry;
    Initializer initializer;
    //public OdometryGlobalCoordinatePosition globalPositionUpdate;

    //Crossbow module
    public DcMotor sideways, vertical, drawback;
    public DcMotor motor1,motor2,motor3,motor4; //starts with left front and moves clockwise
    //public DcMotor verticalLeft, verticalRight, horizontal; //Odometry motors

    public double robotX;
    public double robotY;
    public double robotAngle;

    public Servo lock;

    //Chassis module
    public WebcamName webcamName;

    public BNO055IMU imu;

    // We use millimeters for accuracy
    public final float mmPerInch        = 25.4f;
    public final float mmBotWidth       = 18 * mmPerInch;            // ... or whatever is right for your robot
    public final float mmFTCFieldWidth  = (12 * 12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels
    public final double COUNTS_PER_INCH = (double) mmPerInch * 5; // calculate this with a real number
    public final double COUNTS_PER_REV = COUNTS_PER_INCH;
    public double launchHeight = mmPerInch * 18;

    public RobotHardware(Telemetry telemetry) {
        this.telemetry = telemetry;
        initializer = new Initializer(telemetry, sideways, vertical, drawback, motor1, motor2, motor3, motor4, lock, webcamName, imu);
    }

    public void init(HardwareMap hardwareMap){
        initializer.init(hardwareMap);
    }

    public void move(double x, double y, double r, double power){

        double power1 = y + x + r;
        double power2 = y - x - r;
        double power3 = y + x - r;
        double power4 = y - x + r;

        motor1.setPower(power1 * power);
        motor2.setPower(power2 * power);
        motor3.setPower(power3 * power);
        motor4.setPower(power4 * power);

        telemetry.addData("/> IMU", getRotation("Z"));

    }

    public void moveToAngle(double angle){
        if(getRotation("Z") >= 0) {
            if (getRotation("Z") > angle) {

            }
            if(getRotation("Z") < angle) {

            }
        }
    }

    public double getRotation(String axis){
        Orientation angles = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        switch(axis) {
            case "Z":
                return angles.thirdAngle;
            case "Y":
                return angles.secondAngle;
            case "X":
                return angles.firstAngle;
            default:
                return 0;
        }
    }



    //public void stopGPS(){ globalPositionUpdate.stop(); }
    
}
