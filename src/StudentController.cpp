#include <include/arkheon/character/ICharacterController.h>
#include "PhysicsEngine.h"
#include "PIPDController.h"
#include "AnimationSampler.h"
#include <cstring>
#include <cmath>
#include <cstdio>
#include <algorithm>

namespace
{
    // ===== ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ =====

    struct arkheon_quat_ops
    {
        static arkheon_quat multiply(const arkheon_quat &a, const arkheon_quat &b)
        {
            return {
                a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
                a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
                a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
                a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
        }

        static arkheon_quat conjugate(const arkheon_quat &q)
        {
            return {-q.x, -q.y, -q.z, q.w};
        }

        static arkheon_quat normalize(const arkheon_quat &q)
        {
            float len = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
            if (len < 1e-8f)
                return {0, 0, 0, 1};
            return {q.x / len, q.y / len, q.z / len, q.w / len};
        }

        static void toAxisAngle(const arkheon_quat &q, arkheon_vec3 &axis, float &angle)
        {
            angle = 2.0f * std::acos(std::min(1.0f, std::max(-1.0f, q.w)));
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

        static arkheon_quat fromAxisAngle(const arkheon_vec3 &axis, float angle)
        {
            float half = angle * 0.5f;
            float s = std::sin(half);
            return {axis.x * s, axis.y * s, axis.z * s, std::cos(half)};
        }
    };

    struct arkheon_vec3_ops
    {
        static float length(const arkheon_vec3 &v)
        {
            return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
        }

        static arkheon_vec3 normalize(const arkheon_vec3 &v)
        {
            float len = length(v);
            if (len < 1e-6f)
                return {0, 0, 0};
            return {v.x / len, v.y / len, v.z / len};
        }

        static float dot(const arkheon_vec3 &a, const arkheon_vec3 &b)
        {
            return a.x * b.x + a.y * b.y + a.z * b.z;
        }

        static arkheon_vec3 cross(const arkheon_vec3 &a, const arkheon_vec3 &b)
        {
            return {
                a.y * b.z - a.z * b.y,
                a.z * b.x - a.x * b.z,
                a.x * b.y - a.y * b.x};
        }

        static arkheon_vec3 add(const arkheon_vec3 &a, const arkheon_vec3 &b)
        {
            return {a.x + b.x, a.y + b.y, a.z + b.z};
        }

        static arkheon_vec3 sub(const arkheon_vec3 &a, const arkheon_vec3 &b)
        {
            return {a.x - b.x, a.y - b.y, a.z - b.z};
        }

        static arkheon_vec3 scale(const arkheon_vec3 &v, float s)
        {
            return {v.x * s, v.y * s, v.z * s};
        }

        static float distance(const arkheon_vec3 &a, const arkheon_vec3 &b)
        {
            return length(sub(a, b));
        }
    };

    // ===== PD КОНТРОЛЛЕР =====
    struct PDController
    {
        float kp[10];
        float kd[10];
        arkheon_quat prev_rotation[10];
        bool initialized;

        void initialize(const float *kp_in, const float *kd_in)
        {
            for (int i = 0; i < 10; i++)
            {
                kp[i] = kp_in[i];
                kd[i] = kd_in[i];
                prev_rotation[i] = {0, 0, 0, 1};
            }
            initialized = false;
        }

        void computeTorques(
            const arkheon_quat current_quats[10],
            const arkheon_quat target_quats[10],
            float dt,
            arkheon_vec3 out_torques[10])
        {

            for (int i = 0; i < 10; i++)
            {
                // Error quaternion: rotation from current to target
                arkheon_quat error = arkheon_quat_ops::multiply(
                    target_quats[i],
                    arkheon_quat_ops::conjugate(current_quats[i]));

                // Convert to axis-angle
                arkheon_vec3 axis;
                float angle;
                arkheon_quat_ops::toAxisAngle(error, axis, angle);

                // Angular velocity via finite difference
                arkheon_quat delta_q = arkheon_quat_ops::multiply(
                    current_quats[i],
                    arkheon_quat_ops::conjugate(prev_rotation[i]));
                arkheon_vec3 vel_axis;
                float vel_angle;
                arkheon_quat_ops::toAxisAngle(delta_q, vel_axis, vel_angle);
                float angular_vel = vel_angle / dt;

                // PD torque
                float torque_mag = kp[i] * angle - kd[i] * angular_vel;
                // Clamp torque to prevent explosions
                torque_mag = std::max(-50.0f, std::min(50.0f, torque_mag));

                out_torques[i] = arkheon_vec3_ops::scale(axis, torque_mag);

                // Store for next frame
                prev_rotation[i] = current_quats[i];
            }

            if (!initialized)
            {
                for (int i = 0; i < 10; i++)
                {
                    prev_rotation[i] = current_quats[i];
                }
                initialized = true;
            }
        }
    };

    // ===== ФИЗИЧЕСКИЙ ДВИЖОК =====
    struct PhysicsEngine
    {
        float segment_lengths[10];

        void initialize(const float *segs)
        {
            std::memcpy(segment_lengths, segs, sizeof(float) * 10);
        }

        void integrate(
            const arkheon_quat current_quats[10],
            const arkheon_vec3 torques[10],
            float dt,
            arkheon_bone_override out_overrides[10])
        {

            // Moments of inertia (simplified - proportional to segment length)
            float inertia[10];
            for (int i = 0; i < 10; i++)
            {
                inertia[i] = 0.01f + segment_lengths[i] * 0.05f;
                if (inertia[i] < 0.01f)
                    inertia[i] = 0.01f;
            }

            for (int i = 0; i < 10; i++)
            {
                // Angular acceleration α = τ / I
                float alpha_x = torques[i].x / inertia[i];
                float alpha_y = torques[i].y / inertia[i];
                float alpha_z = torques[i].z / inertia[i];

                // Angular velocity (Euler integration)
                float omega_x = alpha_x * dt;
                float omega_y = alpha_y * dt;
                float omega_z = alpha_z * dt;

                float omega_mag = std::sqrt(omega_x * omega_x + omega_y * omega_y + omega_z * omega_z);

                if (omega_mag > 1e-6f)
                {
                    // Convert delta rotation to quaternion
                    float half = omega_mag * 0.5f;
                    float s = std::sin(half);
                    arkheon_quat delta_q = {
                        (omega_x / omega_mag) * s,
                        (omega_y / omega_mag) * s,
                        (omega_z / omega_mag) * s,
                        std::cos(half)};

                    // Integrate
                    arkheon_quat new_q = arkheon_quat_ops::multiply(delta_q, current_quats[i]);
                    out_overrides[i].local_rotation = arkheon_quat_ops::normalize(new_q);
                }
                else
                {
                    out_overrides[i].local_rotation = current_quats[i];
                }

                out_overrides[i].apply = 1;
            }
        }
    };

    // ===== АНИМАЦИОННЫЙ СЕМПЛЕР =====
    struct AnimationSampler
    {
        float blend_factor;
        int current_clip;
        int target_clip;
        int clip_ids[3] = {12, 47, 83};

        struct MotionData
        {
            float phases[10];
            float amplitudes[10];
        };

        MotionData walk = {
            {0.0f, 0.5f, 0.2f, 0.7f, 0.0f, 0.5f, 0.2f, 0.7f, 0.0f, 0.5f},
            {0.2f, 0.2f, 0.3f, 0.3f, 0.2f, 0.2f, 0.3f, 0.3f, 0.1f, 0.1f}};

        MotionData push = {
            {0.0f, 0.0f, 0.5f, 0.5f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
            {0.1f, 0.1f, 0.4f, 0.4f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}};

        MotionData climb = {
            {0.0f, 0.0f, 0.0f, 0.0f, 0.3f, 0.7f, 0.3f, 0.7f, 0.0f, 0.0f},
            {0.3f, 0.3f, 0.0f, 0.0f, 0.4f, 0.4f, 0.5f, 0.5f, 0.2f, 0.2f}};

        void initialize()
        {
            blend_factor = 1.0f;
            current_clip = 0;
            target_clip = 0;
        }

        void startBlend(int new_clip)
        {
            if (new_clip != current_clip)
            {
                target_clip = new_clip;
                blend_factor = 0.0f;
            }
        }

        void update(float blend_time)
        {
            if (target_clip != current_clip)
            {
                blend_factor = std::min(1.0f, blend_time / 0.2f);
                if (blend_factor >= 1.0f)
                {
                    current_clip = target_clip;
                }
            }
            else
            {
                blend_factor = 1.0f;
            }
        }

        void getReferenceQuaternions(float time, arkheon_quat out_quats[10])
        {
            MotionData *cur = getMotion(current_clip);
            MotionData *tgt = getMotion(target_clip);

            for (int i = 0; i < 10; i++)
            {
                float angle_cur = cur->amplitudes[i] * std::sin(time * 2.0f * 3.14159f + cur->phases[i]);
                float angle_tgt = tgt->amplitudes[i] * std::sin(time * 2.0f * 3.14159f + tgt->phases[i]);

                float angle = (1.0f - blend_factor) * angle_cur + blend_factor * angle_tgt;

                // Convert to quaternion based on joint type
                float half = angle * 0.5f;
                float s = std::sin(half);
                float c = std::cos(half);

                switch (i)
                {
                case ARK_JOINT_UPPERARM_L:
                case ARK_JOINT_UPPERARM_R:
                    out_quats[i] = {s, 0, 0, c};
                    break;
                case ARK_JOINT_LOWERARM_L:
                case ARK_JOINT_LOWERARM_R:
                    out_quats[i] = {0, 0, s, c};
                    break;
                case ARK_JOINT_THIGH_L:
                case ARK_JOINT_THIGH_R:
                    out_quats[i] = {s, 0, 0, c};
                    break;
                case ARK_JOINT_CALF_L:
                case ARK_JOINT_CALF_R:
                    out_quats[i] = {0, 0, s, c};
                    break;
                case ARK_JOINT_FOOT_L:
                case ARK_JOINT_FOOT_R:
                    out_quats[i] = {0, s, 0, c};
                    break;
                default:
                    out_quats[i] = {0, 0, 0, 1};
                }
            }
        }

    private:
        MotionData *getMotion(int clip)
        {
            switch (clip)
            {
            case 0:
                return &walk;
            case 1:
                return &push;
            case 2:
                return &climb;
            default:
                return &walk;
            }
        }
    };

    // ===== ВНУТРЕННЕЕ СОСТОЯНИЕ =====
    struct InternalState
    {
        float segment_lengths[10];

        PDController pd;
        PhysicsEngine physics;
        AnimationSampler anim;

        arkheon_vec3 root_pos = {0, 0, 0};
        arkheon_quat root_rot = {0, 0, 0, 1};
        arkheon_vec3 root_vel = {0, 0, 0};

        int active_motion = 0;
        int next_motion = -1;
        float blend_time = 0.0f;

        arkheon_vec3 com = {0, 0, 0};
        arkheon_vec3 com_vel = {0, 0, 0};

        bool ground_contact = false;
        float ground_y = 0.0f;

        int32_t last_goal_seq = -1;
        arkheon_vec3 goal_path[32];
        int goal_path_len = 0;
        int goal_path_idx = 0;

        float clip_time = 0.0f;
    };

    // HID scancodes
    constexpr uint8_t HID_W = 0x1A;
    constexpr uint8_t HID_A = 0x04;
    constexpr uint8_t HID_S = 0x16;
    constexpr uint8_t HID_D = 0x07;
    constexpr uint8_t HID_SPACE = 0x2C;
    constexpr uint8_t HID_LSHIFT = 0xE1;

} // anonymous namespace

// ===== EXPORTS =====
extern "C"
{

    ARKHEON_CHAR_EXPORT uint32_t arkheon_character_sdk_version(void)
    {
        return ARKHEON_CHARACTER_SDK_VERSION;
    }

    ARKHEON_CHAR_EXPORT const char *arkheon_character_plugin_name(void)
    {
        return "Character Animation Plugin v1.0 - Student";
    }

    ARKHEON_CHAR_EXPORT void arkheon_character_get_motion_clips(
        void * /*handle*/, int32_t out_clip_ids[3])
    {
        out_clip_ids[0] = 12; // walk_forward
        out_clip_ids[1] = 47; // push_two_handed
        out_clip_ids[2] = 83; // climb_low_step
    }

    ARKHEON_CHAR_EXPORT void *arkheon_character_create(const float segment_lengths_m[10])
    {
        auto *state = new InternalState();
        std::memcpy(state->segment_lengths, segment_lengths_m, sizeof(float) * 10);

        float kp[10] = {80, 80, 60, 60, 120, 120, 100, 100, 80, 80};
        float kd[10] = {8, 8, 6, 6, 12, 12, 10, 10, 8, 8};
        state->pd.initialize(kp, kd);
        state->physics.initialize(segment_lengths_m);
        state->anim.initialize();

        return state;
    }

    ARKHEON_CHAR_EXPORT void arkheon_character_destroy(void *handle)
    {
        delete static_cast<InternalState *>(handle);
    }

    ARKHEON_CHAR_EXPORT int32_t arkheon_character_tick(
        void *handle,
        const arkheon_frame *frame,
        const arkheon_bone_state in_bones[66],
        arkheon_bone_override out_overrides[10],
        arkheon_vec3 *out_root_translation_delta,
        arkheon_quat *out_root_rotation_delta,
        const arkheon_input_state *input,
        const arkheon_mission_goal *goal,
        const arkheon_env_api *env)
    {

        if (!handle || !frame || !in_bones || !out_overrides ||
            !out_root_translation_delta || !out_root_rotation_delta)
        {
            return -1;
        }

        auto *state = static_cast<InternalState *>(handle);
        float dt = static_cast<float>(frame->delta_time_s);

        // Default output
        *out_root_translation_delta = {0, 0, 0};
        *out_root_rotation_delta = {0, 0, 0, 1};

        if (frame->is_paused)
            return 0;

        // === 1. ВВОД ===
        if (input)
        {
            // WASD
            float forward = 0, strafe = 0;
            if (input->keys[HID_W])
                forward = 1;
            if (input->keys[HID_S])
                forward = -1;
            if (input->keys[HID_A])
                strafe = -1;
            if (input->keys[HID_D])
                strafe = 1;

            // Sprint
            bool sprint = input->keys[HID_LSHIFT] != 0;
            float speed = sprint ? 4.0f : 1.4f;

            // Hotkeys
            if (input->hotkey_motion_a && state->active_motion != 0)
            {
                state->next_motion = 0;
                state->blend_time = 0.0f;
            }
            if (input->hotkey_motion_b && state->active_motion != 1)
            {
                state->next_motion = 1;
                state->blend_time = 0.0f;
            }
            if (input->hotkey_motion_c && state->active_motion != 2)
            {
                state->next_motion = 2;
                state->blend_time = 0.0f;
            }

            // Локомоция
            float yaw = input->look_yaw_rad;
            float cos_yaw = std::cos(yaw);
            float sin_yaw = std::sin(yaw);

            arkheon_vec3 move_dir = {
                forward * sin_yaw + strafe * cos_yaw,
                0,
                forward * cos_yaw - strafe * sin_yaw};

            float move_len = arkheon_vec3_ops::length(move_dir);
            if (move_len > 0.01f)
            {
                arkheon_vec3 norm_dir = arkheon_vec3_ops::normalize(move_dir);
                state->root_vel = arkheon_vec3_ops::scale(norm_dir, speed);
                *out_root_translation_delta = arkheon_vec3_ops::scale(norm_dir, speed * dt);
            }
            else
            {
                state->root_vel = {0, 0, 0};
            }
        }

        // === 2. АНИМАЦИЯ ===
        state->clip_time += dt;
        state->anim.update(state->blend_time);
        if (state->next_motion >= 0)
        {
            state->blend_time += dt;
            if (state->blend_time >= 0.2f)
            {
                state->active_motion = state->next_motion;
                state->next_motion = -1;
                state->blend_time = 0.0f;
            }
        }

        // === 3. КОНТРОЛЛЕР + ФИЗИКА ===
        arkheon_quat target_quats[10];
        state->anim.getReferenceQuaternions(state->clip_time, target_quats);

        // Get current joint rotations from input bones
        int joint_indices[10] = {9, 33, 10, 34, 56, 61, 57, 62, 58, 63};
        arkheon_quat current_quats[10];
        for (int i = 0; i < 10; i++)
        {
            current_quats[i] = in_bones[joint_indices[i]].local_rotation;
        }

        // PD torques
        arkheon_vec3 torques[10];
        state->pd.computeTorques(current_quats, target_quats, dt, torques);

        // Physics integration
        state->physics.integrate(current_quats, torques, dt, out_overrides);

        // === 4. ЦЕЛИ (GOALS) ===
        if (goal && env && env->report_goal_complete)
        {
            if (goal->sequence_id != state->last_goal_seq)
            {
                state->last_goal_seq = goal->sequence_id;

                if (goal->type == ARK_GOAL_GOTO)
                {
                    if (env->navmesh_query)
                    {
                        state->goal_path_len = env->navmesh_query(
                            env->host_ctx, state->root_pos, goal->target_position,
                            state->goal_path, 32);
                        state->goal_path_idx = 0;
                    }
                }
            }

            // GOTO
            if (goal->type == ARK_GOAL_GOTO)
            {
                float dist = arkheon_vec3_ops::distance(state->root_pos, goal->target_position);
                if (dist < goal->tolerance_m)
                {
                    env->report_goal_complete(env->host_ctx, goal->sequence_id, ARK_GOAL_RESULT_OK);
                }
            }

            // PUSH
            if (goal->type == ARK_GOAL_PUSH && goal->target_object_id >= 0)
            {
                arkheon_vec3 min, max;
                if (env->get_object_aabb(env->host_ctx, goal->target_object_id, &min, &max))
                {
                    arkheon_vec3 obj_center = {
                        (min.x + max.x) * 0.5f,
                        (min.y + max.y) * 0.5f,
                        (min.z + max.z) * 0.5f};
                    float dist = arkheon_vec3_ops::distance(state->root_pos, obj_center);
                    if (dist < 0.5f)
                    {
                        env->report_goal_complete(env->host_ctx, goal->sequence_id, ARK_GOAL_RESULT_OK);
                    }
                }
            }

            // CLIMB
            if (goal->type == ARK_GOAL_CLIMB && goal->target_object_id >= 0)
            {
                arkheon_vec3 min, max;
                if (env->get_object_aabb(env->host_ctx, goal->target_object_id, &min, &max))
                {
                    arkheon_vec3 obj_center = {
                        (min.x + max.x) * 0.5f,
                        (min.y + max.y) * 0.5f,
                        (min.z + max.z) * 0.5f};
                    float xz_dist = std::sqrt(
                        (obj_center.x - state->root_pos.x) * (obj_center.x - state->root_pos.x) +
                        (obj_center.z - state->root_pos.z) * (obj_center.z - state->root_pos.z));
                    if (xz_dist < 0.5f)
                    {
                        out_root_translation_delta->y = 1.0f * dt;
                        env->report_goal_complete(env->host_ctx, goal->sequence_id, ARK_GOAL_RESULT_OK);
                    }
                }
            }

            // PICKUP
            if (goal->type == ARK_GOAL_PICKUP && goal->target_object_id >= 0)
            {
                arkheon_vec3 min, max;
                if (env->get_object_aabb(env->host_ctx, goal->target_object_id, &min, &max))
                {
                    arkheon_vec3 hand = in_bones[10].world_position; // lowerarm_l
                    arkheon_vec3 obj_center = {
                        (min.x + max.x) * 0.5f,
                        (min.y + max.y) * 0.5f,
                        (min.z + max.z) * 0.5f};
                    float dist = arkheon_vec3_ops::distance(hand, obj_center);
                    if (dist < 0.3f)
                    {
                        env->report_goal_complete(env->host_ctx, goal->sequence_id, ARK_GOAL_RESULT_OK);
                    }
                }
            }
        }

        return 0;
    }

} // extern "C"
