#ifndef DIFFDRIVEKINEMATICS_H
#define DIFFDRIVEKINEMATICS_H

class DiffDriveKinematics {
public:
    struct DRIVE_VELOCITIES {
        float linear_mps;
        float omega_radiansps;
    };

    struct WHEEL_VELOCITIES {
        float left_linear_mps;
        float right_linear_mps;
    };

    DiffDriveKinematics(float ptrack_width_m, float pmax_wheel_speed_mps);

    DRIVE_VELOCITIES inverse_kinematics(WHEEL_VELOCITIES wheel_velocities);

    WHEEL_VELOCITIES forward_kinematics(DRIVE_VELOCITIES drive_velocities);

    WHEEL_VELOCITIES saturate_wheel_velocities(WHEEL_VELOCITIES wheel_velocities);

    double get_track_width_m();

private:
    float mtrack_width_m;
    float mmax_wheel_speed_mps;

    float get_top_wheel_velocity_mps(WHEEL_VELOCITIES wheel_velocities);
};

#endif //DIFFDRIVEKINEMATICS_H