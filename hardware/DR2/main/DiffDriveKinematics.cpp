#include "DiffDriveKinematics.h"
#include "math.h"

DiffDriveKinematics::DiffDriveKinematics(float ptrack_width_m, float pmax_wheel_speed_mps) {
    mtrack_width_m = ptrack_width_m;
    mmax_wheel_speed_mps = pmax_wheel_speed_mps;
}

float DiffDriveKinematics::get_top_wheel_velocity_mps(WHEEL_VELOCITIES wheel_velocities) {
    if(wheel_velocities.left_linear_mps > wheel_velocities.right_linear_mps) {
        return wheel_velocities.left_linear_mps;
    } else {
        return wheel_velocities.right_linear_mps;
    }
}

DiffDriveKinematics::WHEEL_VELOCITIES DiffDriveKinematics::saturate_wheel_velocities(WHEEL_VELOCITIES wheel_velocities) {
    float top_wheel_velocity_mps = get_top_wheel_velocity_mps(wheel_velocities);

    return {
        wheel_velocities.left_linear_mps / top_wheel_velocity_mps, 
        wheel_velocities.right_linear_mps / top_wheel_velocity_mps};
}

DiffDriveKinematics::WHEEL_VELOCITIES DiffDriveKinematics::forward_kinematics(
    DiffDriveKinematics::DRIVE_VELOCITIES drive_velocities) {

        float right_linear_mps = 
            drive_velocities.linear_mps + 
            (mtrack_width_m / 2.0) * drive_velocities.omega_radiansps;

        float left_linear_mps = 
            drive_velocities.linear_mps -
            (mtrack_width_m / 2.0) * drive_velocities.omega_radiansps;

        return {left_linear_mps, right_linear_mps};
}

DiffDriveKinematics::DRIVE_VELOCITIES DiffDriveKinematics::inverse_kinematics(
    DiffDriveKinematics::WHEEL_VELOCITIES wheel_velocities) {

        float linear_mps = (1.0 / 2.0) * (
            wheel_velocities.right_linear_mps + 
            wheel_velocities.left_linear_mps);

        float omega_radiansps = ( 1 / mtrack_width_m ) * (
            wheel_velocities.right_linear_mps - 
            wheel_velocities.left_linear_mps );

        return {linear_mps, omega_radiansps};
}

double DiffDriveKinematics::get_track_width_m() {
    return mtrack_width_m;
}