// Arkheon Simulation Technologies
// Proprietary and Confidential.
// arkheon/character/ICharacterController.h
// SDK Version: 1.0
#pragma once
#include <stdint.h>

#define ARKHEON_CHARACTER_SDK_VERSION 0x00010000 // v1.0

#ifdef _WIN32
#define ARKHEON_CHAR_EXPORT __declspec(dllexport)
#else
#define ARKHEON_CHAR_EXPORT __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C"
{
#endif

    /* ─────────── POD math primitives (no STL across boundary) ─────────── */
    typedef struct
    {
        float x, y, z;
    } arkheon_vec3;
    typedef struct
    {
        float x, y, z, w;
    } arkheon_quat; /* glTF order */

    /* ─────────── Bone state (read-only) and override (write) ──────────── */
    typedef struct
    {
        arkheon_quat local_rotation;    /* parent-relative quaternion */
        arkheon_vec3 local_translation; /* parent-relative position (m) */
        arkheon_vec3 world_position;    /* model-space (m), READ-ONLY  */
    } arkheon_bone_state;

    typedef enum
    {
        ARK_JOINT_UPPERARM_L = 0,
        ARK_JOINT_UPPERARM_R = 1,
        ARK_JOINT_LOWERARM_L = 2,
        ARK_JOINT_LOWERARM_R = 3,
        ARK_JOINT_THIGH_L = 4,
        ARK_JOINT_THIGH_R = 5,
        ARK_JOINT_CALF_L = 6,
        ARK_JOINT_CALF_R = 7,
        ARK_JOINT_FOOT_L = 8,
        ARK_JOINT_FOOT_R = 9,
        ARK_JOINT_COUNT = 10
    } arkheon_major_joint;

    typedef struct
    {
        arkheon_quat local_rotation; /* physics output, written back to skeleton */
        uint8_t apply;               /* 0 = use clip, 1 = override with physics */
    } arkheon_bone_override;

    /* ─────────── Frame / clock ─────────────────────────────────────────── */
    typedef struct
    {
        double simulation_time_s;
        double delta_time_s; /* guaranteed 0.02 (50 Hz) */
        uint64_t frame_number;
        uint8_t is_paused;
    } arkheon_frame;

    /* ─────────── Input (keyboard + mouse, host-forwarded) ──────────────── */
    /* Scancodes follow USB HID Usage Table (same as SDL).                  */
    /* Host sets a key index to 1 while held, 0 when released.              */
    typedef struct
    {
        uint8_t keys[256]; /* HID scancode → held state */
        float mouse_dx;    /* pixels since last tick (smoothed) */
        float mouse_dy;
        float look_yaw_rad;      /* accumulated yaw  (host-managed)  */
        float look_pitch_rad;    /* accumulated pitch                */
        uint8_t mouse_buttons;   /* bit0=L, bit1=R, bit2=M           */
        uint8_t hotkey_motion_a; /* edge-triggered: 1 only on press  */
        uint8_t hotkey_motion_b;
        uint8_t hotkey_motion_c;
    } arkheon_input_state;

    /* ─────────── Mission goals (simulation-driven) ─────────────────────── */
    typedef enum
    {
        ARK_GOAL_NONE = 0,
        ARK_GOAL_GOTO = 1,    /* walk to target_position (within tolerance_m) */
        ARK_GOAL_PUSH = 2,    /* push target_object_id along push_dir         */
        ARK_GOAL_CLIMB = 3,   /* climb onto target_object_id (top surface)    */
        ARK_GOAL_PICKUP = 4,  /* pick up target_object_id                     */
        ARK_GOAL_INTERACT = 5 /* generic interact (open door, press button)   */
    } arkheon_goal_type;

    typedef struct
    {
        int32_t sequence_id; /* monotonic; new id => new goal */
        arkheon_goal_type type;
        arkheon_vec3 target_position; /* world-space (m); valid for GOTO */
        int32_t target_object_id;     /* -1 if none */
        arkheon_vec3 push_dir;        /* unit vector; valid for PUSH */
        float tolerance_m;            /* arrival tolerance (default 0.3) */
        float timeout_s;              /* host abandons after this; 0 = none */
    } arkheon_mission_goal;

#define ARK_GOAL_RESULT_OK 0
#define ARK_GOAL_RESULT_FAIL 1
#define ARK_GOAL_RESULT_TIMEOUT 2

    /* ─────────── Environment query API (host callbacks) ────────────────── */
    typedef struct
    {
        void *host_ctx; /* opaque; pass back unchanged to every callback */

        int32_t (*raycast)(void *ctx,
                           arkheon_vec3 origin,
                           arkheon_vec3 dir_normalized,
                           float max_dist_m,
                           arkheon_vec3 *out_hit_world,

                           arkheon_vec3 *out_normal,
                           int32_t *out_object_id);

        int32_t (*get_object_aabb)(void *ctx, int32_t object_id,
                                   arkheon_vec3 *out_min,
                                   arkheon_vec3 *out_max);

        int32_t (*find_object_by_name)(void *ctx, const char *name);

        int32_t (*navmesh_query)(void *ctx,
                                 arkheon_vec3 from, arkheon_vec3 to,
                                 arkheon_vec3 *out_path,
                                 int32_t max_pts);

        void (*report_goal_complete)(void *ctx, int32_t sequence_id, int32_t result);

        arkheon_vec3 (*get_gravity)(void *ctx);
    } arkheon_env_api;

    /* ─────────── Lifecycle exports (ALL must be present) ───────────────── */

    ARKHEON_CHAR_EXPORT uint32_t arkheon_character_sdk_version(void);
    ARKHEON_CHAR_EXPORT const char *arkheon_character_plugin_name(void);

    ARKHEON_CHAR_EXPORT void arkheon_character_get_motion_clips(
        void *handle, int32_t out_clip_ids[3]);

    ARKHEON_CHAR_EXPORT void *arkheon_character_create(
        const float segment_lengths_m[10]);

    ARKHEON_CHAR_EXPORT void arkheon_character_destroy(void *handle);

    ARKHEON_CHAR_EXPORT int32_t arkheon_character_tick(
        void *handle,
        const arkheon_frame *frame,
        const arkheon_bone_state in_bones[66],
        arkheon_bone_override out_overrides[10],
        arkheon_vec3 *out_root_translation_delta,
        arkheon_quat *out_root_rotation_delta,
        const arkheon_input_state *input,
        const arkheon_mission_goal *current_goal, /* may be NULL */
        const arkheon_env_api *env);

#ifdef __cplusplus
} /* extern "C" */
#endif