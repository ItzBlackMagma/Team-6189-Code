package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    }

    public void move(double x, double y, double r, double power){
        //r *= -1;

        double power1 = y + x + r;
        double power2 = y - x - r;
        double power3 = y + x - r;
        double power4 = y - x + r;

        motor1.setPower(power1 * power);
        motor2.setPower(power2 * power);
        motor3.setPower(power3 * power);
        motor4.setPower(power4 * power);

        //printMotorPowers(power1, power2, power3, power4);
    }

    public void translate (double x, double y, double r, double p){
        double angle = getRotation("Z");

        double x1 = x * Math.sin(angle);
        double y1 = x * Math.cos(angle);

        double x2 = y * Math.sin(angle);
        double y2 = y * Math.cos(angle);

        double xPower = x1 + x2;
        double yPower = y1 + y2;

        move(xPower, yPower, r, p);

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


    public void printMotorPowers(double p1, double p2, double p3, double p4){
        telemetry.addData("/> p1", p1);
        telemetry.addData("/> p2", p2);
        telemetry.addData("/> p3", p3);
        telemetry.addData("/> p4", p4);
        telemetry.update();
    }

    //Set everything up
    public void init(HardwareMap hardwareMap){
        telemetry.addData("/> ", "Beginning Initialization Process...");

        initIMU(hardwareMap);

        initChassis(hardwareMap);
        //initBow(hardwareMap);
        //initServos(hardwareMap);

        //webcamName = hardwareMap.get(WebcamName.class, "robo eye");

        telemetry.addData("/> ", "INIT: Webcam Initialized...");
        telemetry.addData("/> ", "Initialization Complete");
        telemetry.update();
    }

    private void initIMU(HardwareMap hardwareMap){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

    }

    private void initChassis(HardwareMap hardwareMap) {
        motor1 = hardwareMap.dcMotor.get("motor1");
        motor2 = hardwareMap.dcMotor.get("motor2");
        motor3 = hardwareMap.dcMotor.get("motor3");
        motor4 = hardwareMap.dcMotor.get("motor4");

        motor1.setDirection(DcMotor.Direction.REVERSE);
        motor2.setDirection(DcMotor.Direction.FORWARD);
        motor3.setDirection(DcMotor.Direction.FORWARD);
        motor4.setDirection(DcMotor.Direction.REVERSE);

        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("/> INIT", "Chassis Motors Initialized");
    }

    private void initBow(HardwareMap hardwareMap){
        sideways = hardwareMap.dcMotor.get("vertical");
        vertical = hardwareMap.dcMotor.get("horizontal");
        drawback = hardwareMap.dcMotor.get("drawback");

        sideways.setDirection(DcMotor.Direction.FORWARD);
        vertical.setDirection(DcMotor.Direction.FORWARD);
        drawback.setDirection(DcMotor.Direction.FORWARD);

        sideways.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drawback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //encoders
        sideways.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        vertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drawback.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("/> ", "INIT: Motors Initialized...");
        telemetry.update();
    }

    private void initServos(HardwareMap hardwareMap){
        lock = hardwareMap.servo.get("lock");
        telemetry.addData("/> ", "INIT: Servos Initialized...");
        telemetry.update();
    }

    //public void stopGPS(){ globalPositionUpdate.stop(); }
    
}
