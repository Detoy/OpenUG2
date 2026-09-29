/* ai.h — OpenUG2 AI module: opponents that follow the game's own racing line
 * (the ROUTES Paths .bin waypoints), with corner-aware pacing and mild
 * rubber-banding toward the player. Also loads/grids a circuit. */
#ifndef OPENUG2_AI_H
#define OPENUG2_AI_H

#include "nfsu2.h"
#include "physics.h"

#define N_AI 4
#define N_TRAFFIC_MAX 16
#define N_ROAM_VISUALS (N_AI + 2) /* reuse four traffic models and two racer models */
#define N_OPENWORLD_AI (N_TRAFFIC_MAX + 2)
/* Keep the original racer slots stable as traffic density changes. */
static inline int ai_traffic_is_racer(int k) { return k>=N_AI && k<N_ROAM_VISUALS; }
static inline int ai_traffic_visual(int k) { return k<N_ROAM_VISUALS?k:k%N_AI; }
/* an AI racer following the racing line */
struct AiCar {
    float pos[3], head, spd, wheel_angle, turn_rate, col[3];
    float half_length, half_width, height;
    float mass; /* tonnes from GLOBALB when available; zero uses neutral mass */
    int t, lap, prevrel, braking;
    float vel[2], steer, target_speed;
    PhysVehicle vehicle;
    PhysRideState ride;
    PhysRideSupport support;
    int ride_ready;
    int render_visible; /* renderer's last conservative frustum test; never simulation input */
};

/* Road nodes read from the game's shared RoutesFreeRoam.bin files. A record's
 * points are ordered; next[] joins consecutive points within that record. */
typedef struct {
    float *xy; int *next, n; float *half_width;
    /* Optional query index built by ai_roads_index (zero = linear scans):
       predecessors per node and a uniform node grid. Never changes results. */
    int *pred_start, *pred_list, *cell_start, *cell_list, gw, gh;
    float gx0, gy0;
} AiRoadNet;
int ai_roads_index(AiRoadNet *roads);
/* Next authored/junction node after `at` coming from `previous` (-1: none). */
int ai_road_next(const AiRoadNet *roads, N2Scene *scene, int at, int previous, float z);
int ai_roads_load(AiRoadNet *roads, const char *tracks_root);
void ai_roads_free(AiRoadNet *roads);

/* Free-roam cars follow the authored road paths, independently of circuits. */
typedef struct {
    int prev, from, to, after;
    float along, travelled, cruise_speed;
    int stop_reason, present;
    unsigned respawns, wait_ticks;
    float lane_offset, max_impact;
    unsigned air_ticks;
} AiTraffic;
typedef struct {
    N2Scene *scene;
    const float (*obst)[4], (*obstz)[2];
    const int *obstsrc;
    int nobst;
    float eye[3], view[2];
    int traffic_target; /* 0..N_TRAFFIC_MAX, roaming racers are independent */
} AiTrafficWorld;
/* Optional read-only observer for behaviour audits; NULL in production. Called
 * once per free-roam simulation tick after contacts and population update. */
typedef void (*AiAuditHook)(const AiCar cars[N_OPENWORLD_AI],
                            const AiTraffic routes[N_OPENWORLD_AI], int n,
                            const AiRoadNet *roads, const AiTrafficWorld *world,
                            const AiCar *player, long tick);
extern AiAuditHook g_ai_audit_hook;
/* Work counters for the optional frame profiler; the caller resets them. */
typedef struct { long passes, pairs, hits, world_fixes, wall_candidates; } AiPerf;
extern AiPerf g_ai_perf;
/* Resolve all live player/AI contacts together; return the player's hit level.
   Pass each car once. player may be NULL for AI-only simulation. */
float ai_car_contacts(AiCar *const cars[], int count, const AiTrafficWorld *world,
                       const AiCar *player);
int ai_traffic_offscreen(const float eye[3], const float view[2],
                         const float pos[3]);
/* Reset/spawn the requested traffic plus two rivals. Returns a slot range,
 * not a live count: inspect routes[k].present when iterating it. */
int ai_traffic_spawn(const AiRoadNet *roads, const AiTrafficWorld *world,
                     AiCar cars[N_OPENWORLD_AI],
                     AiTraffic routes[N_OPENWORLD_AI], const float player[3],
                     float player_heading);
int ai_traffic_respawn(const AiRoadNet *roads, const AiTrafficWorld *world,
                       AiCar cars[N_OPENWORLD_AI],
                       AiTraffic routes[N_OPENWORLD_AI], int k, const float player[3],
                       float player_heading);
/* Reconcile density without removing visible cars. At most one spawn attempt
 * per call; call periodically. Returns the slot range to iterate (holes allowed). */
int ai_traffic_update(const AiRoadNet *roads,const AiTrafficWorld *world,
                       AiCar cars[N_OPENWORLD_AI],AiTraffic routes[N_OPENWORLD_AI],
                       const float player[3],float heading);
void ai_traffic_follow(AiCar cars[N_OPENWORLD_AI], const AiTraffic routes[N_OPENWORLD_AI],
                       const AiRoadNet *roads, int k, int count, const AiCar *player);
void ai_traffic_step(AiCar *car, AiTraffic *route, const AiRoadNet *roads,
                     const AiTrafficWorld *world);

/* (Re)load a racing-line circuit and grid the AI cars on it. Returns the AI
 * count (0 if the file has no usable loop). Frees any previously loaded path,
 * so it is safe to call repeatedly (e.g. when the menu switches circuit).
 * The player spawns at (cx,cy) — the densest built-up spot — facing the line. */
int load_circuit(const char *dataroot, const char *circuit, N2Scene *scene,
                 N2Path *aipath, AiCar *ais, float spawn[3],
                 float *heading0, int *start_idx, float cx, float cy);

/* Use the same shipped circuit line for free-roam rivals without moving or
 * rotating the player. Returns zero in districts with no usable loop. */
int load_roaming_circuit(const char *dataroot, const char *circuit, N2Scene *scene,
                         N2Path *aipath, AiCar *ais, const float player[3],
                         int *start_idx);

/* One tick for one AI: steer toward the next waypoint, pace for the bend
 * ahead, rubber-band toward player_prog (monotonic lap*n+rel progress),
 * follow the ground, count laps. k staggers per-car top speed. */
void ai_step(AiCar *ai, int k, const N2Path *aipath, N2Scene *scene,
             int start_idx, int player_prog);

/* Input-only test driver. Borrows a validated, open XY polyline; never writes
 * the car pose/velocity or queries/snaps its ground height. Speeds: m/tick. */
typedef struct {
    const N2Path *path;
    int segment, direction, stalled, finished, failed;
    float start, progress, length, checkpoint, error, target[2], target_kmh;
} AiDrive;
typedef struct { float throttle, steer; int handbrake; } AiDriveInput;
int ai_drive_route_valid(const N2Path *path);
int ai_drive_init(AiDrive *drive, const N2Path *path, const float pos[3], float heading);
AiDriveInput ai_drive_step(AiDrive *drive, const float pos[3], float heading, float speed);

#endif
