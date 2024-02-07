package frc.robot.utils;

public final class Constants {

    public final static class ID {
        public static final int DRIVETRAIN_FRONT_LEFT = 2;
        public static final int DRIVETRAIN_BACK_LEFT = 3;
        public static final int DRIVETRAIN_FRONT_RIGHT = 0;
        public static final int DRIVETRAIN_BACK_RIGHT = 1;
    }

    public final static class AirplaneLauncher {
        public static final int RIGHT = 5;
        public static final int LEFT = 6;

    }

    public final static class DriveTrain {
        public static final double SLOW_MODE = 0.3;

        public static enum DriveMode {
            ArcadeDrive, TankDrive
        }
    }

    public final static class DualJoystick {
        public static final int X_AXIS = 0;
        public static final int Y_AXIS = 1;
        public static final int DIAL = 2;
        public static final double DEADZONE = 0.2;

    }
}
