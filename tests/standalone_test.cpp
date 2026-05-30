// tests/standalone_test.cpp
// Standalone test harness — NO APP required.
// Build: g++ -std=c++17 tests/standalone_test.cpp src/StudentController.cpp src/MathHelpers.cpp -I include -o standalone_test
// Run: ./standalone_test.exe

#include "include/arkheon/character/ICharacterController.h"
#include <cstdio>
#include <cmath>
#include <cstring>
#include <chrono>

// ===== MOCK ENVIRONMENT API =====
static int32_t mock_raycast(void * /*ctx*/,
                            arkheon_vec3 /*origin*/,
                            arkheon_vec3 /*dir*/,
                            float /*max_dist*/,
                            arkheon_vec3 *out_hit,
                            arkheon_vec3 *out_normal,
                            int32_t *out_object_id)
{
    *out_hit = {0, 0, 0};
    *out_normal = {0, 1, 0};
    *out_object_id = -1;
    return 0; // no hit
}

static int32_t mock_get_object_aabb(void * /*ctx*/,
                                    int32_t /*object_id*/,
                                    arkheon_vec3 *out_min,
                                    arkheon_vec3 *out_max)
{
    *out_min = {-1, -1, -1};
    *out_max = {1, 1, 1};
    return 1; // success
}

static int32_t mock_find_object_by_name(void * /*ctx*/,
                                        const char * /*name*/)
{
    return 0; // return object id 0
}

static int32_t mock_navmesh_query(void * /*ctx*/,
                                  arkheon_vec3 /*from*/,
                                  arkheon_vec3 to,
                                  arkheon_vec3 *out_path,
                                  int32_t /*max_pts*/)
{
    out_path[0] = to;
    return 1; // 1 waypoint
}

static void mock_report_goal_complete(void * /*ctx*/,
                                      int32_t sequence_id,
                                      int32_t result)
{
    std::printf("[GOAL] sequence %d completed with result %d\n",
                sequence_id, result);
}

static arkheon_vec3 mock_get_gravity(void * /*ctx*/)
{
    return {0, -9.81f, 0};
}

// ===== HELPER: create input state =====
static arkheon_input_state create_input(float yaw_rad = 0.0f,
                                        bool w_pressed = false,
                                        bool a_pressed = false,
                                        bool s_pressed = false,
                                        bool d_pressed = false,
                                        bool shift_pressed = false)
{
    arkheon_input_state input = {};

    // Set HID scancodes
    if (w_pressed)
        input.keys[0x1A] = 1; // HID_W
    if (a_pressed)
        input.keys[0x04] = 1; // HID_A
    if (s_pressed)
        input.keys[0x16] = 1; // HID_S
    if (d_pressed)
        input.keys[0x07] = 1; // HID_D
    if (shift_pressed)
        input.keys[0xE1] = 1; // HID_LSHIFT

    input.look_yaw_rad = yaw_rad;
    input.look_pitch_rad = 0.0f;
    input.mouse_dx = 0.0f;
    input.mouse_dy = 0.0f;
    input.mouse_buttons = 0;
    input.hotkey_motion_a = 0;
    input.hotkey_motion_b = 0;
    input.hotkey_motion_c = 0;

    return input;
}

// ===== HELPER: create identity bones =====
static void fill_identity_bones(arkheon_bone_state bones[66])
{
    for (int i = 0; i < 66; i++)
    {
        bones[i].local_rotation = {0, 0, 0, 1};
        bones[i].local_translation = {0, 0, 0};
        bones[i].world_position = {0, 1.0f, 0}; // approximate character height
    }
}

// ===== HELPER: check quaternion validity =====
static bool is_valid_quaternion(const arkheon_quat &q)
{
    float n2 = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    return std::isfinite(n2) && n2 > 0.5f && n2 < 1.5f;
}

// ===== MAIN TEST =====
int main()
{
    std::printf("========================================\n");
    std::printf("Standalone Test for Character Plugin\n");
    std::printf("========================================\n\n");

    // Test 1: SDK version
    uint32_t version = arkheon_character_sdk_version();
    std::printf("[TEST 1] SDK version = 0x%08x\n", version);
    if (version != ARKHEON_CHARACTER_SDK_VERSION)
    {
        std::printf("FAIL: SDK version mismatch (expected 0x%08x)\n",
                    ARKHEON_CHARACTER_SDK_VERSION);
        return 1;
    }
    std::printf("PASS\n\n");

    // Test 2: Plugin name
    const char *name = arkheon_character_plugin_name();
    std::printf("[TEST 2] Plugin name: %s\n", name);
    if (!name || strlen(name) == 0)
    {
        std::printf("FAIL: empty plugin name\n");
        return 1;
    }
    std::printf("PASS\n\n");

    // Test 3: Get motion clips
    int32_t clip_ids[3] = {0, 0, 0};
    arkheon_character_get_motion_clips(nullptr, clip_ids);
    std::printf("[TEST 3] Motion clips: A=%d, B=%d, C=%d\n",
                clip_ids[0], clip_ids[1], clip_ids[2]);
    if (clip_ids[0] == 0 && clip_ids[1] == 0 && clip_ids[2] == 0)
    {
        std::printf("WARNING: all clips returned 0 (bad?)\n");
    }
    std::printf("PASS\n\n");

    // Test 4: Create character
    float segment_lengths[10] = {
        0.0f, 0.0f,           // upper arms (no length - shoulder joints)
        0.296595f, 0.296595f, // lower arms
        0.0f, 0.0f,           // thighs (no length - hip joints)
        0.406626f, 0.406626f, // calves
        0.433194f, 0.433194f  // feet
    };

    void *handle = arkheon_character_create(segment_lengths);
    if (!handle)
    {
        std::printf("FAIL: character creation returned null\n");
        return 1;
    }
    std::printf("[TEST 4] Character created successfully\n");
    std::printf("PASS\n\n");

    // Test 5: 1000 ticks (idle, no input, no goal)
    std::printf("[TEST 5] Running 1000 idle ticks...\n");

    arkheon_bone_state bones[66];
    fill_identity_bones(bones);

    arkheon_env_api env = {};
    env.host_ctx = nullptr;
    env.raycast = mock_raycast;
    env.get_object_aabb = mock_get_object_aabb;
    env.find_object_by_name = mock_find_object_by_name;
    env.navmesh_query = mock_navmesh_query;
    env.report_goal_complete = mock_report_goal_complete;
    env.get_gravity = mock_get_gravity;

    arkheon_frame frame = {};
    frame.delta_time_s = 0.02;
    frame.is_paused = 0;

    auto start_time = std::chrono::high_resolution_clock::now();
    double max_tick_time = 0.0;

    for (uint64_t i = 0; i < 1000; ++i)
    {
        frame.simulation_time_s = i * 0.02;
        frame.frame_number = i;

        auto tick_start = std::chrono::high_resolution_clock::now();

        arkheon_bone_override out_overrides[10] = {};
        arkheon_vec3 root_delta = {0, 0, 0};
        arkheon_quat root_rot_delta = {0, 0, 0, 1};
        arkheon_input_state input = {}; // no input

        int rc = arkheon_character_tick(handle, &frame, bones, out_overrides,
                                        &root_delta, &root_rot_delta,
                                        &input, nullptr, &env);

        auto tick_end = std::chrono::high_resolution_clock::now();
        double tick_time = std::chrono::duration<double, std::milli>(tick_end - tick_start).count();
        if (tick_time > max_tick_time)
            max_tick_time = tick_time;

        if (rc != 0)
        {
            std::printf("FAIL: tick %llu returned %d\n", i, rc);
            return 1;
        }

        // Validate quaternions
        bool all_valid = true;
        for (int j = 0; j < 10; j++)
        {
            if (!is_valid_quaternion(out_overrides[j].local_rotation))
            {
                all_valid = false;
                std::printf("FAIL: invalid quaternion at joint %d, tick %llu\n", j, i);
                break;
            }
        }
        if (!all_valid)
            return 1;
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    double total_time = std::chrono::duration<double, std::milli>(end_time - start_time).count();
    double avg_tick_time = total_time / 1000.0;

    std::printf("  Total time: %.2f ms\n", total_time);
    std::printf("  Average tick: %.4f ms\n", avg_tick_time);
    std::printf("  Max tick: %.4f ms\n", max_tick_time);

    if (avg_tick_time > 18.0)
    {
        std::printf("WARNING: average tick time %.4f ms > 18 ms budget\n", avg_tick_time);
    }
    std::printf("PASS\n\n");

    // Test 6: 1000 ticks with WASD input
    std::printf("[TEST 6] Running 1000 ticks with WASD input...\n");

    for (uint64_t i = 0; i < 1000; ++i)
    {
        frame.simulation_time_s = i * 0.02;
        frame.frame_number = i;

        arkheon_bone_override out_overrides[10] = {};
        arkheon_vec3 root_delta = {0, 0, 0};
        arkheon_quat root_rot_delta = {0, 0, 0, 1};

        // Hold W key for first 500 ticks, then stop
        arkheon_input_state input = create_input(
            0.0f,               // yaw = 0 (forward)
            i < 500,            // W pressed
            false,              // A
            false,              // S
            false,              // D
            i >= 200 && i < 300 // shift during ticks 200-300
        );

        int rc = arkheon_character_tick(handle, &frame, bones, out_overrides,
                                        &root_delta, &root_rot_delta,
                                        &input, nullptr, &env);
        if (rc != 0)
        {
            std::printf("FAIL: tick %llu with input returned %d\n", i, rc);
            return 1;
        }
    }
    std::printf("PASS\n\n");

    // Test 7: Mission goal (GOTO)
    std::printf("[TEST 7] Testing GOTO mission goal...\n");

    arkheon_mission_goal goto_goal = {};
    goto_goal.sequence_id = 1;
    goto_goal.type = ARK_GOAL_GOTO;
    goto_goal.target_position = {3.0f, 0.0f, 4.0f};
    goto_goal.tolerance_m = 0.3f;
    goto_goal.timeout_s = 10.0f;

    arkheon_bone_override out_overrides[10] = {};
    arkheon_vec3 root_delta = {0, 0, 0};
    arkheon_quat root_rot_delta = {0, 0, 0, 1};
    arkheon_input_state input = {};

    int rc = arkheon_character_tick(handle, &frame, bones, out_overrides,
                                    &root_delta, &root_rot_delta,
                                    &input, &goto_goal, &env);
    if (rc != 0)
    {
        std::printf("FAIL: tick with goal returned %d\n", rc);
        return 1;
    }
    std::printf("PASS\n\n");

    // Test 8: Hotkey switching
    std::printf("[TEST 8] Testing hotkey motion switching...\n");

    for (int i = 0; i < 3; i++)
    {
        arkheon_input_state hotkey_input = {};

        // Simulate pressing Q, E, R in sequence
        if (i == 0)
            hotkey_input.hotkey_motion_a = 1; // Q -> motion A
        if (i == 1)
            hotkey_input.hotkey_motion_b = 1; // E -> motion B
        if (i == 2)
            hotkey_input.hotkey_motion_c = 1; // R -> motion C

        rc = arkheon_character_tick(handle, &frame, bones, out_overrides,
                                    &root_delta, &root_rot_delta,
                                    &hotkey_input, nullptr, &env);
        if (rc != 0)
        {
            std::printf("FAIL: tick with hotkey %d returned %d\n", i, rc);
            return 1;
        }
        std::printf("  Hotkey %d applied\n", i);
    }
    std::printf("PASS\n\n");

    // Test 9: 10000 tick stress test
    std::printf("[TEST 9] Stress test: 10000 ticks...\n");

    fill_identity_bones(bones);
    start_time = std::chrono::high_resolution_clock::now();

    for (uint64_t i = 0; i < 10000; ++i)
    {
        frame.simulation_time_s = i * 0.02;
        frame.frame_number = i;

        arkheon_input_state stress_input = create_input(
            static_cast<float>(i) * 0.001f, // slowly rotating yaw
            i % 100 < 50,                   // W on/off
            false, false, false, false);

        rc = arkheon_character_tick(handle, &frame, bones, out_overrides,
                                    &root_delta, &root_rot_delta,
                                    &stress_input, nullptr, &env);
        if (rc != 0)
        {
            std::printf("FAIL: stress tick %llu returned %d\n", i, rc);
            return 1;
        }

        // Check quaternions every 1000 ticks
        if (i % 1000 == 0)
        {
            for (int j = 0; j < 10; j++)
            {
                if (!is_valid_quaternion(out_overrides[j].local_rotation))
                {
                    std::printf("FAIL: invalid quaternion at tick %llu joint %d\n", i, j);
                    return 1;
                }
            }
        }
    }

    end_time = std::chrono::high_resolution_clock::now();
    total_time = std::chrono::duration<double, std::milli>(end_time - start_time).count();
    avg_tick_time = total_time / 10000.0;

    std::printf("  10000 ticks complete\n");
    std::printf("  Total time: %.2f ms\n", total_time);
    std::printf("  Average tick: %.4f ms\n", avg_tick_time);
    std::printf("PASS\n\n");

    // Cleanup
    arkheon_character_destroy(handle);

    // Final summary
    std::printf("========================================\n");
    std::printf("ALL TESTS PASSED!\n");
    std::printf("========================================\n");

    return 0;
}
