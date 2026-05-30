#pragma once
#include <include/arkheon/character/ICharacterController.h>
#include <cmath>
#include <algorithm>


struct PIPDController
{
    float kp[10];
    float kd[10];
    float ki[10];       // Интегральная составляющая
    float integral[10]; // Накопленная ошибка

    void initialize(float *kp_in, float *kd_in, float *ki_in = nullptr)
    {
        for (int i = 0; i < 10; i++)
        {
            kp[i] = kp_in[i];
            kd[i] = kd_in[i];
            ki[i] = (ki_in) ? ki_in[i] : 0.0f;
            integral[i] = 0.0f;
        }
    }

    void compute_torques(
        const arkheon_quat current_quats[10],
        const arkheon_quat target_quats[10],
        const arkheon_vec3 current_angular_vel[10],
        const arkheon_quat previous_rotation[10],
        float dt,
        arkheon_vec3 out_torques[10])
    {
        for (int i = 0; i < 10; i++)
        {
            // Compute error quaternion (rotation from current to target)
            arkheon_quat error = quat_multiply(target_quats[i], quat_conjugate(current_quats[i]));

            // Convert error to axis-angle representation
            arkheon_vec3 error_axis;
            float error_angle;
            quat_to_axis_angle(error, error_axis, error_angle);

            // Compute angular velocity error
            arkheon_quat delta_q = quat_multiply(current_quats[i], quat_conjugate(previous_rotation[i]));
            float delta_angle;
            arkheon_vec3 delta_axis;
            quat_to_axis_angle(delta_q, delta_axis, delta_angle);
            arkheon_vec3 angular_velocity = vec3_scale(vec3_normalize(delta_axis), delta_angle / dt);

            // Интегральная составляющая (anti-windup)
            integral[i] += error_angle * dt;
            integral[i] = std::max(-1.0f, std::min(1.0f, integral[i])); // Clamp

            // PIPD control law
            out_torques[i] = {
                kp[i] * error_axis.x * error_angle - kd[i] * angular_velocity.x + ki[i] * error_axis.x * integral[i],
                kp[i] * error_axis.y * error_angle - kd[i] * angular_velocity.y + ki[i] * error_axis.y * integral[i],
                kp[i] * error_axis.z * error_angle - kd[i] * angular_velocity.z + ki[i] * error_axis.z * integral[i]};
        }
    }

private:
    static arkheon_quat quat_multiply(const arkheon_quat &a, const arkheon_quat &b)
    {
        return {
            a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
            a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
    }

    static arkheon_quat quat_conjugate(const arkheon_quat &q)
    {
        return {-q.x, -q.y, -q.z, q.w};
    }

    static void quat_to_axis_angle(const arkheon_quat &q, arkheon_vec3 &axis, float &angle)
    {
        angle = 2.0f * std::acos(static_cast<double>(std::min(1.0f, std::max(-1.0f, q.w))));
        float sin_half = std::sqrt(1.0f - q.w * q.w);
        if (sin_half > 1e-6f)
        {
            axis = {q.x / sin_half, q.y / sin_half, q.z / sin_half};
        }
        else
        {
            axis = {1, 0, 0};
        }
    }

    static arkheon_vec3 vec3_normalize(const arkheon_vec3 &v)
    {
        float len = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
        if (len < 1e-6f)
            return {0, 0, 0};
        return {v.x / len, v.y / len, v.z / len};
    }

    static arkheon_vec3 vec3_scale(const arkheon_vec3 &v, float s)
    {
        return {v.x * s, v.y * s, v.z * s};
    }
};
