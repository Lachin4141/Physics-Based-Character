#pragma once
#include <include/arkheon/character/ICharacterController.h>
#include <cmath>
#include <cstring>

struct PhysicsEngine
{
    float segment_lengths[10];

    void initialize(const float *segs)
    {
        std::memcpy(segment_lengths, segs, sizeof(float) * 10);
    }

    void step_physics(
        const arkheon_quat current_quats[10],
        const arkheon_vec3 torques[10],
        const arkheon_vec3 &center_of_mass,
        bool is_ground_contact,
        float dt,
        arkheon_bone_override out_overrides[10])
    {
        constexpr float MOMENTS_OF_INERTIA[10] = {
            0.01f, 0.01f,   // shoulders
            0.008f, 0.008f, // elbows
            0.05f, 0.05f,   // hips
            0.03f, 0.03f,   // knees
            0.005f, 0.005f  // ankles
        };

        for (int i = 0; i < 10; i++)
        {
            // Angular acceleration from torque (simplified: τ = I·α)
            arkheon_vec3 angular_acc = {
                torques[i].x / MOMENTS_OF_INERTIA[i],
                torques[i].y / MOMENTS_OF_INERTIA[i],
                torques[i].z / MOMENTS_OF_INERTIA[i]};

            // Angular velocity update (simplified Euler step)
            float angular_vel_x = angular_acc.x * dt;
            float angular_vel_y = angular_acc.y * dt;
            float angular_vel_z = angular_acc.z * dt;

            // Convert angular velocity to quaternion delta
            float angle = std::sqrt(angular_vel_x * angular_vel_x +
                                    angular_vel_y * angular_vel_y +
                                    angular_vel_z * angular_vel_z);

            if (angle > 1e-6f)
            {
                float half_angle = angle * 0.5f;
                float sin_half = std::sin(half_angle);
                arkheon_quat delta_q = {
                    (angular_vel_x / angle) * sin_half,
                    (angular_vel_y / angle) * sin_half,
                    (angular_vel_z / angle) * sin_half,
                    std::cos(half_angle)};

                // Integrate quaternion
                arkheon_quat new_quat = quat_multiply(delta_q, current_quats[i]);

                // Normalize to prevent drift
                float len = std::sqrt(new_quat.x * new_quat.x + new_quat.y * new_quat.y +
                                      new_quat.z * new_quat.z + new_quat.w * new_quat.w);
                if (len > 0)
                {
                    out_overrides[i].local_rotation = {
                        new_quat.x / len,
                        new_quat.y / len,
                        new_quat.z / len,
                        new_quat.w / len};
                }
                else
                {
                    out_overrides[i].local_rotation = current_quats[i];
                }
            }
            else
            {
                out_overrides[i].local_rotation = current_quats[i];
            }

            out_overrides[i].apply = 1;
        }
    }

private:
    // Вспомогательные функции (должны быть определены или подключены)
    static arkheon_quat quat_multiply(const arkheon_quat &a, const arkheon_quat &b)
    {
        return {
            a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
            a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
    }
};
