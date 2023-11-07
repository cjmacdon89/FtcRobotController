package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

@Autonomous(name="Auto_Two_Cone_Right", group="")
public class Auto_Two_Cone_Right extends LinearOpMode {

    ElapsedTime timmer = new ElapsedTime();
    DcMotor LB_Drive;
    DcMotor RB_Drive;
    DcMotor LF_Drive;
    DcMotor RF_Drive;
    DcMotor Elevator;
    Servo Claw;
    DcMotor DeadWheel_x;
    DcMotor DeadWheel_y;
    double power = 0.4;
    double park = 0;
    double distanceSensor = 500;
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, correction;
    // Camera initalization
    OpenCvCamera camera;
    static final double FEET_PER_METER = 3.28084;
    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    // UNITS ARE METERS
    double tagsize = 0.10;
    int ID_TAG_OF_INTEREST = 18; // Tag ID 18 from the 36h11 family
    int Tag_seen = 0;
    AprilTagDetection tagOfInterest = null;
    double current_position = 0;
    double Elevator_Position = 0;
    //Encoder stuff
    double DW_x_Position = 0;
    double DW_y_Position = 0;
    double Tick_Conv = (50 * 3.1416)/2048;
    double DW_x_Dis;
    double DW_y_Dis;

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Motor Mapping
        LB_Drive = hardwareMap.get(DcMotor.class, "LB_Drive");
        RB_Drive = hardwareMap.get(DcMotor.class, "RB_Drive");
        LF_Drive = hardwareMap.get(DcMotor.class, "LF_Drive");
        RF_Drive = hardwareMap.get(DcMotor.class, "RF_Drive");
        Elevator = hardwareMap.get(DcMotor.class, "Elevator");
        DeadWheel_x = hardwareMap.get(DcMotor.class, "DeadWheel_x");
        DeadWheel_y = hardwareMap.get(DcMotor.class, "DeadWheel_y");
        Claw = hardwareMap.get(Servo.class, "Claw");

        //setting the direction of motors
        LB_Drive.setDirection(DcMotor.Direction.FORWARD);
        RB_Drive.setDirection(DcMotor.Direction.REVERSE);
        LF_Drive.setDirection(DcMotor.Direction.FORWARD);
        RF_Drive.setDirection(DcMotor.Direction.FORWARD);
        Claw.setPosition(0.38);

        // Setup Gyro
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        // Retrieve and initialize the IMU.
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();
        telemetry.addData("Mode", "waiting for start");
        telemetry.update();
        telemetry.setMsTransmissionInterval(50);

        // Camera Initalization
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        Claw.setPosition(0.38);
        // wait for start button.
            sleep(20);

        telemetry.addData("Mode", "running");
        telemetry.update();

        left_drive(700, power, 0);
        stop_robot();

        //start cone code
        Elevator_Position = 1800;
        Elevator.setTargetPosition((int) Elevator_Position);
        Elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        current_position = Elevator.getCurrentPosition();
        Elevator.setPower(1.0);
        sleep(1500);

        forward_drive(400, power, 0);//1000
        stop_robot();

        Elevator_Position = 0;
        Elevator.setTargetPosition((int)Elevator_Position);
        Elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        current_position = Elevator.getCurrentPosition();
        Elevator.setPower(1.0);

        sleep(1000);
        Claw.setPosition(0.15);
        sleep(250);
        Elevator.setPower(0);

        backward_drive(200, power, 0);//
        stop_robot();

        turnright_drive(700, 0.6, 0);//600
        stop_robot();

        forward_drive(1000, power, 90);
        stop_robot();

        right_drive_time(500, power, 90);
        stop_robot();

        sleep(200);

        left_drive(4800, 0.5, 90);
        stop_robot();

        forward_drive(1800, power, 90);
        stop_robot();

        Elevator_Position = 800;
        Elevator.setTargetPosition((int)Elevator_Position);
        Elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        current_position = Elevator.getCurrentPosition();
        Elevator.setPower(1.0);
        sleep(500);

        forward_drive_time(450, power, 90);
        stop_robot();
        sleep(650);
        Claw.setPosition(0.38);
        sleep(500);

        Elevator_Position = 1800;
        Elevator.setTargetPosition((int)Elevator_Position);
        Elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        current_position = Elevator.getCurrentPosition();
        Elevator.setPower(1.0);
        sleep(500);

        backward_drive(2400, power, 90);
        stop_robot();
        right_drive(1100, power, 90);
        stop_robot();
        forward_drive(225, power, 90);
        stop_robot();
        Elevator_Position = 0;
        Elevator.setTargetPosition((int)Elevator_Position);
        Elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        current_position = Elevator.getCurrentPosition();
        Elevator.setPower(1.0);

        sleep(1000);
        Claw.setPosition(0.15);
        sleep(250);
        Elevator.setPower(0);

        backward_drive(425, power, 90);
        stop_robot();

        right_drive(625, power, 90);
        stop_robot();


        if (Tag_seen == 1){
            backward_drive(900, 0.9, 90);
            stop_robot();
        }
        else if (Tag_seen == 3){
            forward_drive(900, 0.9, 90);
            stop_robot();
        }
        else{
            stop_robot();
        }

    }

    // Function to get angle change from gyro
    private double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     *
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection() {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .02;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    private double checkDirection_90() {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .02;

        angle = getAngle();

        if (angle == 90)
            correction = 0;             // no adjustme/nt.
        else
            correction = -angle - 90;        // reverse sign of angle for correction.


        correction = correction * gain;

        return correction;
    }

    private double checkDirection_180() {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .04;

        angle = getAngle_180();

        if (angle == 180)
            correction = 0;             // no adjustment.
        else
            correction = 180 - angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    private double getAngle_180() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = (angles.firstAngle) - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    private void forward_drive(double Distane_travel, double power, double direction) {
        DW_y_Position = -1*(DeadWheel_y.getCurrentPosition()*Tick_Conv);
        Distane_travel = Distane_travel + DW_y_Position;
        while (DW_y_Position < Distane_travel) {
            if (direction == 0) {
                correction = checkDirection();
            } else {
                correction = checkDirection_90();
            }
            LB_Drive.setPower(-power + correction);
            RB_Drive.setPower(-power - correction);
            LF_Drive.setPower(-power + correction);
            RF_Drive.setPower(-power - correction);
            DW_x_Position = -1*(DeadWheel_x.getCurrentPosition()*Tick_Conv);
            DW_y_Position = -1*(DeadWheel_y.getCurrentPosition()*Tick_Conv);
        }
    }

    private void backward_drive(double Distane_travel, double power, double direction) {
        DW_y_Position = -1*(DeadWheel_y.getCurrentPosition()*Tick_Conv);
        Distane_travel = (-1*Distane_travel) + DW_y_Position;
        while (DW_y_Position > Distane_travel) {
            if (direction == 0) {
                correction = checkDirection();
            } else {
                correction = checkDirection_90();
            }
            LB_Drive.setPower(power + correction);
            RB_Drive.setPower(power - correction);
            LF_Drive.setPower(power + correction);
            RF_Drive.setPower(power - correction);
            DW_x_Position = -1*(DeadWheel_x.getCurrentPosition()*Tick_Conv);
            DW_y_Position = -1*(DeadWheel_y.getCurrentPosition()*Tick_Conv);
        }
    }

    private void right_drive(double Distane_travel, double power, double direction) {
        DW_x_Position = -1*(DeadWheel_x.getCurrentPosition()*Tick_Conv);
        Distane_travel = Distane_travel + DW_x_Position;
        while (DW_x_Position < Distane_travel) {
            if (direction == 0) {
                correction = checkDirection();
            } else {
                correction = checkDirection_90();
            }
            LB_Drive.setPower(power + correction);
            RB_Drive.setPower(-power - correction);
            LF_Drive.setPower(-power + correction);
            RF_Drive.setPower(power - correction);
            DW_x_Position = -1*(DeadWheel_x.getCurrentPosition()*Tick_Conv);
            DW_y_Position = -1*(DeadWheel_y.getCurrentPosition()*Tick_Conv);
        }
    }

    private void left_drive(double Distane_travel, double power, double direction) {
        DW_x_Position = -1*(DeadWheel_x.getCurrentPosition()*Tick_Conv);
        Distane_travel = (-1*Distane_travel)  + DW_x_Position;
        while (DW_x_Position > Distane_travel) {
            if (direction == 0) {
                correction = checkDirection();
            } else {
                correction = checkDirection_90();
            }
            LB_Drive.setPower(-power + correction);
            RB_Drive.setPower(power - correction);
            LF_Drive.setPower(power + correction);
            RF_Drive.setPower(-power - correction);
            DW_x_Position = -1*(DeadWheel_x.getCurrentPosition()*Tick_Conv);
            DW_y_Position = -1*(DeadWheel_y.getCurrentPosition()*Tick_Conv);
        }
    }

    private void turnright_drive(double run_time, double power, double direction) {
        timmer.reset();
        while (timmer.milliseconds() < run_time) {
            LB_Drive.setPower((power * -1));
            RB_Drive.setPower((power * 1));
            LF_Drive.setPower((power * -1));
            RF_Drive.setPower((power * 1));
        }
    }

    private void turnleft_drive(double run_time, double power, double direction) {
        timmer.reset();
        while (timmer.milliseconds() < run_time) {
            LB_Drive.setPower((power * 1));
            RB_Drive.setPower((power * -1));
            LF_Drive.setPower((power * 1));
            RF_Drive.setPower((power * -1));
        }
    }

    private void right_drive_time(double run_time, double power, double direction) {
        timmer.reset();
        while (timmer.milliseconds() < run_time) {
            if (direction == 0) {
                correction = checkDirection();
            } else {
                correction = checkDirection_90();
            }
            LB_Drive.setPower(power + correction);
            RB_Drive.setPower(-power - correction);
            LF_Drive.setPower(-power + correction);
            RF_Drive.setPower(power - correction);
        }
    }

    private void forward_drive_time(double run_time, double power, double direction) {
        timmer.reset();
        while (timmer.milliseconds() < run_time) {
            if (direction == 0) {
                correction = checkDirection();
            } else {
                correction = checkDirection_90();
            }
            LB_Drive.setPower(-power + correction);
            RB_Drive.setPower(-power - correction);
            LF_Drive.setPower(-power + correction);
            RF_Drive.setPower(-power - correction);
        }
    }

    private void stop_robot() {
        LB_Drive.setPower(0);
        RB_Drive.setPower(0);
        LF_Drive.setPower(0);
        RF_Drive.setPower(0);
    }
}