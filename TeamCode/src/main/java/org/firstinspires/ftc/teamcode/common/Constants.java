package org.firstinspires.ftc.teamcode.common;

import com.arcrobotics.ftclib.controller.PIDController;

public class Constants {
    // Game Constants
    public static final double END_GAME_TIME = 80.0;  // last 40 seconds
    public static final double FINAL_TIME = 110.0;    // last 10 seconds

    // Arm Lift PID Controller Init.
    public static final double up_Kp = 0.0055, up_Ki = 0, up_Kd = 0.00025;
    public static final double down_Kp = 0.00038, down_Ki = 0, down_Kd = 0.00001;
    public static PIDController armUpPID = new PIDController(up_Kp, up_Ki, up_Kd);
    public static PIDController armDownPID = new PIDController(down_Kp, down_Ki, down_Kd);
    public static final double f = 0.045, l = 0;
    public static final double ARM_LIFT_TICKS_PER_DEGREE = 28.0 * 5.23 * 5.23 * 3.61 / 360;


    // goBILDA 5203 Series Yellow Jacket Planetary Gear Motor
    // max encoder ticks per second
    // https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/
    public static final double VELOCITY_CONSTANT = 537.7 * 312/60;

    // Define Arm Constants: Power
    public static final double ARM_SWIVEL_MAX_POWER = 0.35;
    public static final double ARM_LIFT_MAX_POWER = 0.5;
    public static final double ARM_LIFT_MIN_POWER = 0.01;
    public static final double ARM_LIFT_POWER_AT_REST = 0.12;
    public static final double ARM_EXTENSION_MAX_POWER = 0.6;

    // Define Arm Constants: Encoder and Servo Values
    public static final int ARM_SWIVEL_RIGHT_LIMIT = -70;
    public static final int ARM_SWIVEL_LEFT_LIMIT = 160;

    public static final int ARM_LIFT_MAX_POSITION = 910;
    public static final int ARM_LIFT_MIN_POSITION = 20;

    public static final double CLAW_MOVE_INCREMENT = 0.025;
    public static final double CLAW_LIFT_MIN_RANGE = 0.0;
    public static final double CLAW_LIFT_MAX_RANGE = 1.0;
    public static final double CLAW_LIFT_START_POSITION = 0.6;   // scaled, see MIN and MAX_RANGE

    public static final double CLAW_GRAB_MIN_RANGE = 0.1;
    public static final double CLAW_GRAB_MAX_RANGE = 0.54;
    public static final double CLAW_GRAB_OPEN_POSITION = 0.3;
    public static final double CLAW_GRAB_CLOSE_POSITION = 1.0;

    // Define Arm Constants: Preset Junction Positions
    public static final int ARM_LIFT_GROUND_POSITION = 20, ARM_EXTEND_GROUND_POSITION = -1825; public static final double CLAW_LIFT_GROUND_POSITION = 0.55;
    public static final int ARM_LIFT_LOW_POSITION = 360, ARM_EXTEND_LOW_POSITION = -10; public static final double CLAW_LIFT_LOW_POSITION = 0.55;
    public static final int ARM_LIFT_MED_POSITION = 660, ARM_EXTEND_MED_POSITION = -10; public static final double CLAW_LIFT_MED_POSITION = 0.50;
    public static final int ARM_LIFT_HIGH_POSITION = 880, ARM_EXTEND_HIGH_POSITION = -3215; public static final double CLAW_LIFT_HIGH_POSITION = 0.35;

    // Telemetry
    public static final String TELEMETRY_SEPARATOR = "--------------------";
}
