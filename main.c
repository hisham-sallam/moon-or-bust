// Clean the code
// Add landing thank you

#include <stdio.h>
#include <math.h>

#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"
#include "glad.h"
#include "stdbool.h"

#define RAYGUI_IMPLEMENTATION
#include "raygui.h"  
#include "styles/turbo/style_turbo.h"

#define GRAVITIONAL_CONSTANT (double) 6.6743e-11
#define MOON_MASS (double) 7.346e22
#define EARTH_MASS (double) 5.9722e24
#define MOON_RADIUS (double) 1753500
#define EARTH_RADIUS (double) 6.371e6

#define LANDER_TORQUE 75000 //Newtons
#define LANDER_DAMPING_RATE 70000.0
#define DRY_MASS (float) 4500.0f
#define STARTING_WET_MASS (float) 15000.0f
#define ISP (int) 450

#define X_HAT CLITERAL(Vector3) { 1.0, 0.0, 0.0 }
#define Y_HAT CLITERAL(Vector3) { 0.0, 1.0, 0.0 }
#define Z_HAT CLITERAL(Vector3) { 0.0, 0.0, 1.0 }

#define LOCAL_PITCH_AXIS X_HAT
#define LOCAL_ROLL_AXIS Y_HAT
#define LOCAL_YAW_AXIS Z_HAT

#define RENDER_SCALE 10
#define MOON_MESH_RADIUS_XZ 0.987231
#define MOON_MESH_RADIUS_Y  0.997577

#define MAX_PARTICLES (int)10000

typedef enum{
    MAX_NUM_OF_TEXTURES = 7,
    MAX_NUM_OF_MODELS = 4
} MAX_ITEMS ;

typedef enum{
    SHIP_MODEL = 0,
    MOON_MODEL = 1,
    TERRAIN_MODEL = 2,
    NAVBALL_MODEL = 3,
} MODEL_ID ;

typedef enum{
    PROGRADE_TEXTURE = 0,
    RETROGRADE_TEXTURE = 1,
    RADIAL_OUT_TEXTURE = 2,
    RADIAL_IN_TEXTURE = 3,
    NORMAL_TEXTURE = 4,
    ANTI_NORMAL_TEXTURE = 5,
    POINTER_TEXTURE = 6 
} TEXTURE_ID ;

typedef struct vec_3 {
    double x;
    double y;
    double z;
} vec_3;

typedef struct Vessel {

    // Assets
    Model model;

    // Positions
    vec_3 position;
    Vector3 display_pos;

    // Box Model
    float width; 
    float depth;
    float height;

    // Orientation
    Quaternion rotation;
    Vector3 angular_velocity;
    float pitch_inertia;
    float roll_inertia;
    float yaw_inertia;

    // Mass & engine specifications
    double dry_mass;
    double wet_mass;
    double isp;
    double velocity_exhaust;
    double delta_v;
    double fuel_rate; // in kgs per second
    float engine_throttle;

    // Orbital Variables
    double apoapsis;
    double periapsis;
    vec_3 velocity;
    vec_3 accelration;

    // Extras
    bool isInfiniteFuel;
    bool is_landing;
    bool landed;

    Vector3 collision_shape_points[8]
} Vessel;

typedef struct Terrain {
    Model model;
    Vector3 position; // we only use display position here because it is its actual location
    BoundingBox collision_box;
} Terrain;

typedef struct CelestialBody {

    Model model;
    vec_3 position;
    Vector3 display_pos;

} CelestialBody;

typedef struct ParticleSystem {

    Mesh particle_shape;
    Shader particle_shader;
    Material particle_material;
    Matrix *transforms;

} ParticleSystem;

typedef struct GameInstance {

    Texture2D *textures[MAX_NUM_OF_TEXTURES];
    Model *models[MAX_NUM_OF_MODELS];

    Matrix *particle_transforms;
    ParticleSystem engine_particles;

    Vessel lander;
    CelestialBody the_moon;
    Terrain lunar_terrain;

    Camera3D world_camera;
    Camera3D navball_camera;
    Camera2D ui_camera;

} GameInstance;

float random_float(float min, float max) {
    return min + ((float)rand() / (float)RAND_MAX) * (max - min);
}

inline double clamp(double value, double min, double max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

double vec_3Magnitude(vec_3 v) {
    return sqrt((v.x*v.x) + (v.y*v.y) + (v.z*v.z));
}

vec_3 vec_3Normalize( vec_3 v ) {

    vec_3 result = {0.0, 0.0, 0.0};

    double length = vec_3Magnitude(v);
    double ilength;

    if (length == 0.0) length = 1.0;

    ilength = 1.0/length;

    v.x *= ilength;
    v.y *= ilength;
    v.z *= ilength;
    result = v;

    return result;
}

vec_3 vec_3CrossProduct(vec_3 v1, vec_3 v2) {
    return (vec_3){
        v1.y*v2.z - v1.z*v2.y,
        v1.z*v2.x - v1.x*v2.z,
        v1.x*v2.y - v1.y*v2.x
    };
}

static double calculateExhaustVelocity(double isp) {
    return (double) isp * (GRAVITIONAL_CONSTANT*EARTH_MASS)/ (EARTH_RADIUS*EARTH_RADIUS);
}

static double calculateDeltaV(double isp, double wet_mass, double dry_mass) {
    return (double) calculateExhaustVelocity(isp) * log(wet_mass/dry_mass);
}

static vec_3 calculateStartingVelocity(vec_3* pos, double apo, double peri, bool is_inclined) {

    *pos = (vec_3){ MOON_RADIUS+apo, 0.0, 0.0 };
    double semi_major_axis = (MOON_RADIUS+apo+MOON_RADIUS+peri)/2.0;

    if (is_inclined) {
        return (vec_3) {0,sqrt(GRAVITIONAL_CONSTANT*MOON_MASS*((2/(MOON_RADIUS+apo))-(1/semi_major_axis))),0};
    }
    else {
        return (vec_3) {0,0,sqrt(GRAVITIONAL_CONSTANT*MOON_MASS*((2/(MOON_RADIUS+apo))-(1/semi_major_axis)))};
    }
}

static double calculateTangentialVelocity(vec_3 vel, vec_3 pos) {

    double pos_mag = vec_3Magnitude(pos);
    vec_3 h = vec_3CrossProduct(vel,pos);
    double h_mag = vec_3Magnitude(h);

    return h_mag / pos_mag;
}

static double calculateRadialVelocity(vec_3 vel, vec_3 pos) {
    double pos_mag = vec_3Magnitude(pos);
    return (double) (pos.x*vel.x + pos.y*vel.y + pos.z*vel.z) / pos_mag;
}

static double calculateOrbibtalInclination(vec_3 vel, vec_3 pos) {
    vec_3 h = vec_3CrossProduct(pos,vel);
    double h_mag = vec_3Magnitude(h);
    return acos(h.y/h_mag) * RAD2DEG;
}
static double calculateOrbitalHeight(vec_3 ship_pos) {
    return vec_3Magnitude(ship_pos) - MOON_RADIUS;
}

// Since this function honestly looks a bit confusing, its puropose is simply calculate
// the rotation relative to the world Up in degreees (radians in this case)
static float calculateLanderThetaFromQuaternion(Vessel *ship, Vector3 local_up) {
    Vector3 ship_up = Vector3RotateByQuaternion(Z_HAT, ship->rotation);
    float dot = Vector3DotProduct(ship_up, local_up);
    return acosf(fmaxf(-1.0f, fminf(1.0f, dot)));
}

static float calculateTippingWidth(Vessel *ship, Vector3 local_up) {
    float min_projection = Vector3DotProduct(ship->collision_shape_points[0], local_up);
    float max_projection = min_projection;

    for (int i = 0; i < 8; i++) {
        float projection = Vector3DotProduct(ship->collision_shape_points[i], local_up);
        if (projection < min_projection) min_projection = projection;
        if (projection > max_projection) max_projection = projection;
    }
    return max_projection - min_projection;
}

static vec_3 calculateOrbitalAcceleration(vec_3 pos) {
    double distance = sqrt(pos.x*pos.x + pos.y*pos.y + pos.z*pos.z);
    if (distance == 0.0) {
        return (vec_3) {0.0, 0.0, 0.0};
    }
    double accel_mag = -(GRAVITIONAL_CONSTANT*MOON_MASS)/(pow(distance,3));
    return (vec_3) { accel_mag * pos.x, accel_mag * pos.y, accel_mag * pos.z};
}

static Vessel resetShip(Vessel ship) {

    double new_apoapsis = 2000;
    double new_periapsis = 2000;
    vec_3 new_position;
    vec_3 new_velocity = calculateStartingVelocity(&new_position,new_apoapsis,new_periapsis,false);
    Vector3 new_display_pos = { new_position.x/RENDER_SCALE, new_position.y/RENDER_SCALE, new_position.z/RENDER_SCALE };
    vec_3 new_accelration = calculateOrbitalAcceleration(new_position);

    ship.position = new_position;
    ship.display_pos = new_display_pos;
    ship.velocity = new_velocity;
    ship.accelration = new_accelration;
    ship.apoapsis = new_apoapsis;
    ship.periapsis = new_periapsis;

    return ship;
}

static bool landingLegCollision(Vessel* ship, CelestialBody* body, Terrain *terrain){

    Vector3 corners[8];
    BoundingBox box = GetModelBoundingBox(ship->model);

    corners[0] = (Vector3){ box.min.x, box.min.y, box.min.z }; // Bottom-Left-Back
    corners[1] = (Vector3){ box.max.x, box.min.y, box.min.z }; // Bottom-Right-Back
    corners[2] = (Vector3){ box.max.x/2.0f, box.max.y/2.0f, box.min.z/2.0f }; // Top-Right-Back
    corners[3] = (Vector3){ box.min.x/2.0f, box.max.y/2.0f, box.min.z/2.0f }; // Top-Left-Back
    corners[4] = (Vector3){ box.min.x, box.min.y, box.max.z }; // Bottom-Left-Front
    corners[5] = (Vector3){ box.max.x, box.min.y, box.max.z }; // Bottom-Right-Front
    corners[6] = (Vector3){ box.max.x/2.0f, box.max.y/2.0f, box.max.z/2.0f }; // Top-Right-Front
    corners[7] = (Vector3){ box.min.x/2.0f, box.max.y/2.0f, box.max.z/2.0f }; // Top-Left-Fron

    Matrix scale = MatrixScale(1.0/RENDER_SCALE, 1.0/RENDER_SCALE, 1.0/RENDER_SCALE);

    Vector3 axis;
    float angle;
    QuaternionToAxisAngle(ship->rotation,&axis,&angle);

    Matrix rotation = MatrixRotate(axis, angle);
    Matrix translate = MatrixTranslate(ship->display_pos.x, ship->display_pos.y, ship->display_pos.z);
    Matrix transform = MatrixMultiply(MatrixMultiply(scale, rotation), translate);

    Vector3 rel_pos = Vector3Subtract(
        (Vector3){
            (float)ship->position.x,
            (float)ship->position.y,
            (float)ship->position.z
        },
        (Vector3){
            (float)body->position.x,
            (float)body->position.y,
            (float)body->position.z
        }
    );

    Vector3 up = Vector3Normalize(rel_pos);
    float height = (float)(vec_3Magnitude(ship->position) - MOON_RADIUS);
    Vector3 plane_pos = Vector3Scale(up, -height / RENDER_SCALE);
    float plane_dist = Vector3DotProduct(plane_pos, up);

    for (int i = 0; i < 8; i++) {
        corners[i] = Vector3Transform(corners[i], transform);
        ship->collision_shape_points[i] = corners[i];
        float corner_dist = Vector3DotProduct(ship->collision_shape_points[i], up); 

        if (corner_dist <= plane_dist) {
            if (calculateTangentialVelocity(ship->velocity,ship->velocity) > 2.1 || calculateRadialVelocity(ship->velocity, ship->position) > 3.0) {
                *ship = resetShip(*ship);
            }
            return true; 
        }
    }
    
    return false;
}


static void calculateOrbitalVelocity(double dt, Vessel* ship, Terrain *terrain, CelestialBody *body) {

    ship->position.x += ship->velocity.x * dt + 0.5 * ship->accelration.x * (dt * dt);
    ship->position.y += ship->velocity.y * dt + 0.5 * ship->accelration.y * (dt * dt);
    ship->position.z += ship->velocity.z * dt + 0.5 * ship->accelration.z * (dt * dt);

    vec_3 old_accel = ship->accelration;

    if (landingLegCollision(ship,body,terrain) && ship->is_landing) {
        ship->landed = true;
        ship->velocity = (vec_3){0.0, 0.0, 0.0};
        ship->accelration = (vec_3){0.0, 0.0, 0.0};
        old_accel = (vec_3){0.0, 0.0, 0.0};
    }
    else {
        ship->landed = false;
        ship->accelration = calculateOrbitalAcceleration(ship->position);
    }

    if (ship->wet_mass > ship->dry_mass && ship->engine_throttle > 0.0f) {

        Vector3 body_thrust = Vector3RotateByQuaternion(Z_HAT, ship->rotation);

        vec_3 world_thrust = {
            (double)body_thrust.x,
            (double)body_thrust.y,
            (double)body_thrust.z
        };

        double thrust_dir_mag = sqrt(world_thrust.x * world_thrust.x +
                                     world_thrust.y * world_thrust.y +
                                     world_thrust.z * world_thrust.z);
        if (thrust_dir_mag > EPSILON) {
            world_thrust.x /= thrust_dir_mag;
            world_thrust.y /= thrust_dir_mag;
            world_thrust.z /= thrust_dir_mag;
        }

        double engine_thrust = (ship->velocity_exhaust * ship->fuel_rate) *
                               (ship->engine_throttle / 100.0);
        double accel_mag = engine_thrust / ship->wet_mass;

        ship->accelration.x += world_thrust.x * accel_mag;
        ship->accelration.y += world_thrust.y * accel_mag;
        ship->accelration.z += world_thrust.z * accel_mag;

        if (!ship->isInfiniteFuel){
            ship->wet_mass -= ship->fuel_rate * (ship->engine_throttle / 100.0) * dt;
        }
        if (ship->wet_mass < ship->dry_mass) ship->wet_mass = ship->dry_mass;
    }
    
    ship->velocity.x += 0.5 * (old_accel.x + ship->accelration.x) * dt;
    ship->velocity.y += 0.5 * (old_accel.y + ship->accelration.y) * dt;
    ship->velocity.z += 0.5 * (old_accel.z + ship->accelration.z) * dt;
    
}

static void updateOrbitalElements(Vessel* ship) {
    double mu = GRAVITIONAL_CONSTANT * MOON_MASS;

    double r_mag = vec_3Magnitude(ship->position);
    double v_mag = vec_3Magnitude(ship->velocity);

    vec_3 h = vec_3CrossProduct(ship->position,ship->velocity);

    double inclination = calculateOrbibtalInclination(ship->velocity,ship->position);

    vec_3 v_cross_h = {
        ship->velocity.y*h.z - ship->velocity.z*h.y,
        ship->velocity.z*h.x - ship->velocity.x*h.z,
        ship->velocity.x*h.y - ship->velocity.y*h.x
    };
    vec_3 e_vec = {
        (v_cross_h.x/mu) - (ship->position.x/r_mag),
        (v_cross_h.y/mu) - (ship->position.y/r_mag),
        (v_cross_h.z/mu) - (ship->position.z/r_mag)
    };
    double eccentricity = sqrt(e_vec.x*e_vec.x + e_vec.y*e_vec.y + e_vec.z*e_vec.z);

    double specific_energy = (v_mag*v_mag)/2.0 - mu/r_mag;
    double semi_major_axis = -mu / (2.0 * specific_energy);

    double r_apo  = semi_major_axis * (1.0 + eccentricity);
    double r_peri = semi_major_axis * (1.0 - eccentricity);

    ship->delta_v = calculateDeltaV(ISP,ship->wet_mass,ship->dry_mass);
    ship->apoapsis  = r_apo  - MOON_RADIUS;
    ship->periapsis = r_peri - MOON_RADIUS;

}

static Vector3 calculateInertia(float mass, float depth, float width, float height) {
    float pitch = 1./12. * mass*( (width*width) + (height*height) );
    float roll = 1./12. * mass*( (depth*depth) + (height*height) );
    float yaw = 1./12. * mass*( (width*width) + (depth*depth) );
    return (Vector3) { pitch, roll, yaw };
}

static Vector3 torqueInput() {
    Vector3 result = { 0 , 0 , 0 };

    // Pitch
    if (IsKeyDown(KEY_W)) result.x += LANDER_TORQUE;
    if (IsKeyDown(KEY_S)) result.x -= LANDER_TORQUE;

    // Roll
    if (IsKeyDown(KEY_D)) result.y += LANDER_TORQUE;
    if (IsKeyDown(KEY_A)) result.y -= LANDER_TORQUE;

    // Yaw
    if (IsKeyDown(KEY_Q)) result.z += LANDER_TORQUE;
    if (IsKeyDown(KEY_E)) result.z -= LANDER_TORQUE;

    return result;
}

static void applyDamping(Vector3 *torque, Vector3 angular_velocity) {
    bool pitch_input = IsKeyDown(KEY_W) || IsKeyDown(KEY_S);
    bool roll_input = IsKeyDown(KEY_D) || IsKeyDown(KEY_A);
    bool yaw_input = IsKeyDown(KEY_Q) || IsKeyDown(KEY_E);

    float pitch_rate = Vector3DotProduct(angular_velocity,LOCAL_PITCH_AXIS);
    float roll_rate = Vector3DotProduct(angular_velocity,LOCAL_ROLL_AXIS);
    float yaw_rate = Vector3DotProduct(angular_velocity,LOCAL_YAW_AXIS);
    //printf("%lf, %lf, %lf\n",pitch_rate,roll_rate,yaw_rate);

    if(!pitch_input) torque->x -= LANDER_DAMPING_RATE * pitch_rate;
    if(!roll_input) torque->y -= LANDER_DAMPING_RATE * roll_rate;
    if(!yaw_input) torque->z -= LANDER_DAMPING_RATE * yaw_rate;
}

static void updateRotation(double dt, Vessel *ship) {

    // Used to contain all torque inputs
    Vector3 torque = { 0, 0, 0 };

    if (ship->landed) {
        Vector3 local_up = (Vector3){
            (float)(ship->position.x / vec_3Magnitude(ship->position)),
            (float)(ship->position.y / vec_3Magnitude(ship->position)),
            (float)(ship->position.z / vec_3Magnitude(ship->position))
        };

        float current_angle = calculateLanderThetaFromQuaternion(ship, local_up);
        const float UPRIGHT_THRESHOLD = 2.0f * DEG2RAD;

        if (current_angle > UPRIGHT_THRESHOLD && current_angle < 90*DEG2RAD) {
            float tipping_width = calculateTippingWidth(ship, local_up);
            Vector3 ship_up = Vector3RotateByQuaternion(Z_HAT, ship->rotation);
            Vector3 tipping_axis = Vector3Normalize(Vector3CrossProduct(local_up, ship_up));

            if (Vector3Length(tipping_axis) > EPSILON) {
                float radius = 0.5f * ship->height;
                float inertia = (1.0f/3.0f) * ship->wet_mass *
                                (powf(tipping_width, 2.0f) + powf(ship->height, 2.0f));
                float gravity = (GRAVITIONAL_CONSTANT * EARTH_MASS) / (EARTH_RADIUS * EARTH_RADIUS);

                Vector3 lean_dir_world = Vector3Normalize(Vector3CrossProduct(tipping_axis, local_up));
                Vector3 lean_dir_local = Vector3RotateByQuaternion(lean_dir_world, QuaternionInvert(ship->rotation));

                float half_width  = ship->width * 0.5f;
                float half_depth  = ship->depth * 0.5f;
                float half_height = ship->height * 0.5f;

                float footprint_half_extent = fabsf(lean_dir_local.x) * half_width +
                                            fabsf(lean_dir_local.y) * half_depth;

                float critical_angle = atan2f(footprint_half_extent, half_height);

                float torque_val = ship->wet_mass * gravity * radius *
                                    sinf(current_angle - critical_angle);
                float angular_accel = torque_val / inertia;

                Vector3 tipping_axis_local = Vector3RotateByQuaternion(
                    tipping_axis, QuaternionInvert(ship->rotation));

                float current_rate = Vector3DotProduct(ship->angular_velocity, tipping_axis_local);
                current_rate += angular_accel * dt;
                current_rate *= 0.98f;

                ship->angular_velocity = Vector3Scale(tipping_axis_local, current_rate);
            }
        }
        // Probably shouldnt let this function handle it but this handles crashing on the ground
        else if (current_angle > 90.0f *DEG2RAD) {
            *ship = resetShip(*ship);
        } 
        else {
            ship->angular_velocity = (Vector3){0.0f, 0.0f, 0.0f};
        }
    }
    else {
        torque = torqueInput();
        applyDamping(&torque, ship->angular_velocity);
    }

    // Applying the rule A = Torque / Inertia
    Vector3 angular_accelration = Vector3Add( Vector3Scale( LOCAL_PITCH_AXIS, torque.x/ship->pitch_inertia ) , 
                                              Vector3Add( Vector3Scale( LOCAL_ROLL_AXIS, torque.y/ ship->roll_inertia ) , Vector3Scale( LOCAL_YAW_AXIS, torque.z/ ship->yaw_inertia ) ));
    // Adding to the velocity
    ship->angular_velocity = Vector3Add( ship->angular_velocity, Vector3Scale( angular_accelration, dt ));
    float radians_per_second = Vector3Length(ship->angular_velocity);

    if (radians_per_second > EPSILON) {
        Vector3 axis = Vector3Scale(ship->angular_velocity, 1.0f/radians_per_second);
        float delta_angle = radians_per_second * dt;

        Quaternion delta_q = QuaternionFromAxisAngle(axis,delta_angle);

        ship->rotation = QuaternionMultiply(ship->rotation,delta_q);
        // Normalizing it as a failsafe
        ship->rotation = QuaternionNormalize(ship->rotation);
    }
}

static void loadTextures(Texture2D *textures[MAX_NUM_OF_TEXTURES]) {
    for (int i = 0; i < MAX_NUM_OF_TEXTURES; i++){
        textures[i] = malloc(sizeof(Texture2D));
    }
    *textures[PROGRADE_TEXTURE] = LoadTexture("assets/prograde.png");
    *textures[RETROGRADE_TEXTURE] = LoadTexture("assets/retrograde.png");
    *textures[RADIAL_IN_TEXTURE] = LoadTexture("assets/radialout.png");
    *textures[RADIAL_OUT_TEXTURE] = LoadTexture("assets/radialin.png");
    *textures[NORMAL_TEXTURE] = LoadTexture("assets/normal.png");
    *textures[ANTI_NORMAL_TEXTURE] = LoadTexture("assets/antinormal.png");
    *textures[POINTER_TEXTURE] = LoadTexture("assets/pointer.png");
}

static void loadModels(Model *models[MAX_NUM_OF_MODELS]) {
    for (int i = 0; i < MAX_NUM_OF_MODELS; i++){
        models[i] = malloc(sizeof(Model));
    }
    *models[SHIP_MODEL] = LoadModel("assets/lunar_lander.glb");
    *models[MOON_MODEL] = LoadModel("assets/the_moon.glb");
    *models[TERRAIN_MODEL] = LoadModel("assets/plane.glb");
    *models[NAVBALL_MODEL] = LoadModel("assets/navball.glb");
}

static void initNavball(Texture2D *textures[MAX_NUM_OF_TEXTURES], Model *models[MAX_NUM_OF_MODELS], Camera3D *camera, Vessel *ship, CelestialBody *body, RenderTexture2D *navball_texture) {
    static Vector3 heading = { 0.0f, 0.0f, -1.0f }; 

    const float MIN_SPEED       = 1.0f;  
    const float MIN_HORIZ_SPEED = 1.0f;

    Vector3 rel_pos = Vector3Subtract(
    (Vector3){ ship->position.x, ship->position.y, ship->position.z },
    (Vector3){ body->position.x, body->position.y, body->position.z });
    Vector3 up = Vector3Normalize(rel_pos);

    Vector3 vel = (Vector3){ (float)ship->velocity.x, (float)ship->velocity.y, (float)ship->velocity.z };
    float speed = Vector3Length(vel);

    Vector3 v_h = Vector3Subtract(vel, Vector3Scale(up, Vector3DotProduct(vel, up)));
    float vh_len = Vector3Length(v_h);

    if (vh_len > MIN_HORIZ_SPEED) {
        heading = Vector3Scale(v_h, 1.0f / vh_len);
    } else {
        Vector3 h = Vector3Subtract(heading, Vector3Scale(up, Vector3DotProduct(heading, up)));
        float hl = Vector3Length(h);
        if (hl > 1e-4f) {
            heading = Vector3Scale(h, 1.0f / hl);
        } else {
            Vector3 ref = (fabsf(up.y) < 0.99f) ? (Vector3){0, 1, 0} : (Vector3){1, 0, 0};
            heading = Vector3Normalize(Vector3CrossProduct(up, ref));
        }
    }

    Vector3 right   = Vector3Normalize(Vector3CrossProduct(heading, up));
    Vector3 forward = Vector3CrossProduct(right, up);

    Matrix orbital_frame = {
        right.x,   up.x,   forward.x,   0.0f,
        right.y,   up.y,   forward.y,   0.0f,
        right.z,   up.z,   forward.z,   0.0f,
        0.0f,      0.0f,   0.0f,        1.0f
    };

    Quaternion orbital_rot = QuaternionFromMatrix(orbital_frame);

    Quaternion relative_lander_rot = QuaternionMultiply(QuaternionInvert(orbital_rot), ship->rotation);
    Quaternion navball_rotation = QuaternionInvert(relative_lander_rot);

    Vector3 navball_axis;
    float angle;
    QuaternionToAxisAngle(navball_rotation, &navball_axis, &angle);

    Quaternion inv_lander = QuaternionInvert(ship->rotation);

    BeginTextureMode(*navball_texture);
        ClearBackground(BLANK);
        BeginMode3D(*camera);
            DrawModelEx(*models[NAVBALL_MODEL], (Vector3){0.0f, 0.0f, 0.0f}, navball_axis, angle * RAD2DEG, (Vector3){1.0f, 1.0f, 1.0f}, WHITE);
            glClear(GL_DEPTH_BUFFER_BIT);
            if (speed > MIN_SPEED) {
                Vector3 vhat = Vector3Scale(vel, 1.0f / speed);
                if (Vector3DotProduct(Vector3RotateByQuaternion(vhat, inv_lander),Z_HAT) > 0) {
                    DrawBillboard(*camera, *textures[PROGRADE_TEXTURE],Vector3RotateByQuaternion(vhat, inv_lander), 0.7f, WHITE);
                }
                else {
                    DrawBillboard(*camera, *textures[RETROGRADE_TEXTURE],Vector3RotateByQuaternion(Vector3Scale(vhat, -1), inv_lander), 0.7f, WHITE);
                }
            }

            Vector3 r = Vector3RotateByQuaternion(up, inv_lander);
            if (Vector3DotProduct(Vector3Scale(r,-1),Z_HAT) < 0) { 
                DrawBillboard(*camera, *textures[RADIAL_IN_TEXTURE], r, 0.7f, WHITE);
            }
            else {
                DrawBillboard(*camera, *textures[RADIAL_OUT_TEXTURE],Vector3RotateByQuaternion(Vector3Scale(up, -1), inv_lander), 0.7f, WHITE);
            }
            
            if (vh_len > MIN_HORIZ_SPEED) {
                Vector3 n = Vector3RotateByQuaternion(right, inv_lander);
                if (Vector3DotProduct(n,Z_HAT) > 0) {
                    DrawBillboard(*camera, *textures[NORMAL_TEXTURE], n, 0.7f, WHITE);
                }
                else {
                    DrawBillboard(*camera, *textures[ANTI_NORMAL_TEXTURE], Vector3Scale(n, -1), 0.7f, WHITE);
                }
            }

            DrawBillboard(*camera, *textures[POINTER_TEXTURE], (Vector3){0, 0, 1}, 1.0f, WHITE);
        EndMode3D();
    EndTextureMode();
}

static void simulateEngineParticles(Vessel *ship, ParticleSystem *engine_particle) {
    Vector3 engine_effects_direction = Vector3RotateByQuaternion(Z_HAT, ship->rotation);
    engine_effects_direction = Vector3Scale(engine_effects_direction,-1.7);

    for (int i = 0; i < ship->engine_throttle*(MAX_PARTICLES/100); i++) {

        // the funny bell curve equation
        float max_bell_height = 1;
        float current_height = random_float(0, max_bell_height);
        float sigma = 2;
        float max_radius_at_height = sigma * sqrtf(2 * logf(max_bell_height/(current_height+0.3)));

        current_height -= random_float(0.0,0.1); // add heigh variation so it doesnt look like a cone

        // becase we are dealing with a negative curve
        float flipped_height = max_bell_height - current_height;

        Vector3 random = {random_float(-10,10),random_float(-10,10),random_float(-10,10)};
        random = Vector3Normalize(random);

        Vector3 height_scale = Vector3Add(engine_effects_direction,Vector3Scale(engine_effects_direction,flipped_height));

        Vector3 perpendicular = Vector3Normalize(Vector3CrossProduct(Vector3Normalize(height_scale),random));
        perpendicular = Vector3Scale(perpendicular,max_radius_at_height);
        perpendicular = Vector3Add(perpendicular,height_scale);

        Matrix translation = MatrixTranslate(perpendicular.x, perpendicular.y,perpendicular.z);
                                    
        Vector3 axis = Vector3Normalize((Vector3){ (float)0, (float)180, (float)0 });
        float angle = (float)GetRandomValue(0, 180)*DEG2RAD;
        Matrix rotation = MatrixRotate(axis, angle);

        engine_particle->transforms[i] = MatrixMultiply(rotation, translation);
    }
}

static void cameraMovement(Camera3D* world_camera, CelestialBody* body, Vessel* ship, Terrain *terrain) {
    static float orbitRadius = 5.0f/RENDER_SCALE;
    static float minRadius = 5.0f/RENDER_SCALE;
    static float maxRadius = 100.0f/RENDER_SCALE;

    static float azimuth = 0.0f;
    static float elevation = 0.3f;

    const float elevationLimit = 1.5f;
    const float mouseSensitivity = 0.005f;
    const float zoomSpeed = 5.0f/RENDER_SCALE;

    Vector3 localUp;
    Vector3 localRight;
    Vector3 localForward;

    if (ship->is_landing) {

        Vector3 ship_position = {
            (float)ship->position.x,
            (float)ship->position.y,
            (float)ship->position.z
        };

        localUp = Vector3Normalize(ship_position);

        Vector3 reference;

        if (fabsf(localUp.y) < 0.9f) {
            reference = Y_HAT;
        }
        else {
            reference = X_HAT;
        }

        localRight = Vector3Normalize(Vector3CrossProduct(reference, localUp));
        localForward = Vector3Normalize(Vector3CrossProduct(localUp, localRight));
    }
    else {
        localUp = Vector3Normalize(Vector3Subtract(ship->display_pos,body->display_pos));
        Vector3 reference = Y_HAT;

        if (fabsf(Vector3DotProduct(reference, localUp)) > 0.99f) {
            reference = X_HAT;
        }

        localRight = Vector3Normalize(Vector3CrossProduct(reference, localUp));
        localForward = Vector3Normalize(Vector3CrossProduct(localUp, localRight));
    }

    if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {

        Vector2 delta = GetMouseDelta();

        azimuth -= delta.x * mouseSensitivity;
        elevation += delta.y * mouseSensitivity;

        if (ship->is_landing) {
            elevation = Clamp( elevation,0.0f,elevationLimit);
        }
        else {
            elevation = Clamp(elevation,-elevationLimit,elevationLimit);
        }

    }

    float wheel = GetMouseWheelMove();
    if (wheel != 0.0f) {
        orbitRadius -= wheel * zoomSpeed;
        orbitRadius = Clamp(orbitRadius,minRadius,maxRadius);
    }

    float ce = cosf(elevation);
    float se = sinf(elevation);

    float ca = cosf(azimuth);
    float sa = sinf(azimuth);

    Vector3 camera_offset = { 0 };

    camera_offset = Vector3Add(camera_offset,Vector3Scale(localRight,ca * ce));
    camera_offset = Vector3Add(camera_offset,Vector3Scale(localForward,sa * ce));
    camera_offset = Vector3Add( camera_offset,Vector3Scale( localUp, se));
    camera_offset = Vector3Scale( camera_offset, orbitRadius );

    Vector3 render_ship_pos = {0.0f,0.0f,0.0f};

    world_camera->position = Vector3Add(render_ship_pos,camera_offset);

    world_camera->target = render_ship_pos;
    world_camera->up = localUp;

    body->display_pos = (Vector3){
        (float)(body->position.x - ship->position.x) / RENDER_SCALE,
        (float)(body->position.y - ship->position.y) / RENDER_SCALE,
        (float)(body->position.z - ship->position.z) / RENDER_SCALE
    };

    ship->display_pos = render_ship_pos;
}

static void throttleControl(Vessel* ship) {
    if (IsKeyPressed(KEY_Z)) {
        ship->engine_throttle = 100.0f;
    }
    if (IsKeyPressed(KEY_X)) {
        ship->engine_throttle = 0.0f;
    }

    if (IsKeyDown(KEY_LEFT_CONTROL)) {
        ship->engine_throttle -= 0.5f;
        ship->engine_throttle = Clamp(ship->engine_throttle,0.0f,100.0f);
    }
    if (IsKeyDown(KEY_LEFT_SHIFT)) {
        ship->engine_throttle += 0.5f;
        ship->engine_throttle = Clamp(ship->engine_throttle,0.0f,100.0f);
    }
}

static void drawWorld(Model *models[MAX_NUM_OF_MODELS], Vessel *ship, CelestialBody *body, Camera3D * world_camera, ParticleSystem *particles) {
    float ship_angle;
    Vector3 ship_axis;
    float moon_angle;
    Vector3 moon_axis;
    QuaternionToAxisAngle(ship->rotation, &ship_axis, &ship_angle);
    QuaternionToAxisAngle(QuaternionIdentity(),&moon_axis,&moon_angle);
    BeginMode3D(*world_camera);
        ClearBackground(BLACK);
        if (sqrt(ship->position.x*ship->position.x +
            ship->position.y*ship->position.y +
            ship->position.z*ship->position.z) <= MOON_RADIUS+100.0f) {
            ship->is_landing = true;
        }
        else { 
            DrawModelEx(body->model ,
                        body->display_pos,
                        moon_axis,
                        moon_angle * RAD2DEG,
                        (Vector3){(MOON_RADIUS/RENDER_SCALE)/MOON_MESH_RADIUS_XZ, (MOON_RADIUS/RENDER_SCALE)/MOON_MESH_RADIUS_Y, (MOON_RADIUS/RENDER_SCALE)/MOON_MESH_RADIUS_XZ},
                        WHITE);
            ship->is_landing = false;
        }
    EndMode3D();
    BeginMode3D(*world_camera);
    rlPushMatrix();
        rlScalef(0.1,0.1,0.1);
        DrawModelEx(ship->model,
            ship->display_pos,
            ship_axis,
            ship_angle * RAD2DEG,
            (Vector3){1.0, 1.0,1.0},
            WHITE);
                
        BeginBlendMode(BLEND_ALPHA);
            if (ship->wet_mass > ship->dry_mass) {
                DrawMeshInstanced(particles->particle_shape,particles->particle_material,particles->transforms,ship->engine_throttle*(MAX_PARTICLES/100));
            }
        EndBlendMode();
        
        rlPopMatrix();
        Vector3 rel_pos = Vector3Subtract(
            (Vector3){
                (float)ship->position.x,
                (float)ship->position.y,
                (float)ship->position.z
            },
            (Vector3){
                (float)body->position.x,
                (float)body->position.y,
                (float)body->position.z
            }
        );

        Vector3 up = Vector3Normalize(rel_pos);
        float height = (float)(vec_3Magnitude(ship->position) - MOON_RADIUS);
        Vector3 plane_pos = Vector3Scale(up, -height / RENDER_SCALE);

        Quaternion q = QuaternionFromVector3ToVector3(Y_HAT,up);

        Vector3 axis;
        float angle;

        QuaternionToAxisAngle(q,&axis,&angle);
        
        if (ship->is_landing == true) {

            DrawModelEx(*models[TERRAIN_MODEL],plane_pos,axis,angle * RAD2DEG,(Vector3){10000.0f,10000.0f,10000.0f},WHITE);

        }
    EndMode3D();
}

static bool initToolWindow(Vessel* ship) {

    GuiSetStyle(DEFAULT, TEXT_SIZE, 16);

    Rectangle windowBounds = { 50, 50, 360, 380 };

    static char apoapsis_buffer[32];
    static char periapsis_buffer[32];
    static char delta_v_buffer[32];
    static bool edit_apoapsis  = false;
    static bool edit_periapsis = false;
    static bool edit_delta_v = false;
    bool isInfiniteFuel = ship->isInfiniteFuel;
    static bool is_inclined = false;

    if (GuiWindowBox(windowBounds, "Tools")) {
        return false;
    }

    float start_x = windowBounds.x + 15;
    float start_y = windowBounds.y + 35;

    start_y += 45;
    GuiCheckBox((Rectangle){ start_x, start_y, 20, 20 }, " Inclined Orbit", &is_inclined);
    
    start_y += 45;
    GuiLabel((Rectangle){ start_x, start_y, 100, 20 }, "Apoapsis (m):");
    if (GuiTextBox((Rectangle){ start_x + 100, start_y, 120, 24 }, apoapsis_buffer, 32, edit_apoapsis)) edit_apoapsis = !edit_apoapsis;

    start_y += 45;
    GuiLabel((Rectangle){ start_x, start_y, 100, 20 }, "Periapsis (m):");
    if (GuiTextBox((Rectangle){ start_x + 100, start_y, 120, 24 }, periapsis_buffer, 32, edit_periapsis)) edit_periapsis = !edit_periapsis;

    if (GuiButton((Rectangle){ start_x + 230, start_y, 100, 24 }, "Set Orbit")) {
        double val_apo  = atof(apoapsis_buffer);
        double val_peri = atof(periapsis_buffer);

        if (!isfinite(val_apo))  val_apo  = 2000.0;
        if (!isfinite(val_peri)) val_peri = 2000.0;

        if (val_apo  < 2000.0)     val_apo  = 2000.0;
        if (val_apo  > 59490000.0) val_apo  = 59490000.0;
        if (val_peri < 2000.0)     val_peri = 2000.0;
        if (val_peri > 59490000.0) val_peri = 59490000.0;

        if (val_peri > val_apo) {
            double tmp = val_apo;
            val_apo = val_peri;
            val_peri = tmp;
        }

        snprintf(apoapsis_buffer,  sizeof(apoapsis_buffer),  "%.0f", val_apo);
        snprintf(periapsis_buffer, sizeof(periapsis_buffer), "%.0f", val_peri);

        ship->velocity    = calculateStartingVelocity(&ship->position, val_apo, val_peri, is_inclined);
        ship->accelration = calculateOrbitalAcceleration(ship->position);
    }

    start_y += 45;
    GuiCheckBox((Rectangle){ start_x, start_y, 20, 20 }, " Enable Infinite Fuel", &ship->isInfiniteFuel);

    start_y += 45;
    GuiLabel((Rectangle){ start_x, start_y, 100, 20 }, "Set Delta V:");
    if (GuiTextBox((Rectangle){ start_x + 100, start_y, 120, 24 }, delta_v_buffer, 32, edit_delta_v)) edit_delta_v = !edit_delta_v;

    if (GuiButton((Rectangle){ start_x + 230, start_y, 100, 24 }, "Set Value")) {
        double val_delta_v  = atof(delta_v_buffer);

        if (!isfinite(val_delta_v))  val_delta_v  = calculateDeltaV(ISP,STARTING_WET_MASS,DRY_MASS);

        if (val_delta_v  < 0.0) val_delta_v  = 0.0;
        if (val_delta_v  >  calculateDeltaV(ISP,STARTING_WET_MASS,DRY_MASS)) val_delta_v  =  calculateDeltaV(ISP,STARTING_WET_MASS,DRY_MASS);

        snprintf(delta_v_buffer,  sizeof(delta_v_buffer),  "%.0f", val_delta_v);
        ship->wet_mass = DRY_MASS * exp(val_delta_v / calculateExhaustVelocity(ISP));
    }

    GuiSetStyle(DEFAULT, TEXT_SIZE, 32);
    return true;
}

static void initalizeUI(Texture2D *textures[MAX_NUM_OF_TEXTURES], Model *models[MAX_NUM_OF_MODELS], Camera2D *UI_camera, Camera3D *navball_cam, Vessel* ship, CelestialBody *body, RenderTexture2D *navball_texture, int *warp) {

    Vector2 navball_pos = { GetScreenWidth()/2.0f-100, GetScreenHeight()-200 };

    initNavball(textures,models,navball_cam,ship,body,navball_texture);

    BeginMode2D(*UI_camera);

        GuiSetStyle(DEFAULT, TEXT_SIZE, 32);
        
        Rectangle navball_source_rec = { 0.0f, 0.0f, (float)navball_texture->texture.width, -(float)navball_texture->texture.height };
        Rectangle navball_dest_rec = { navball_pos.x, navball_pos.y, (float)200, (float)200 };
        Rectangle main_deck = {navball_dest_rec.x-50, navball_dest_rec.y-50, navball_dest_rec.width+100,navball_dest_rec.height+50};
        DrawRectangle(main_deck.x, main_deck.y, main_deck.width ,main_deck.height,(Color){ 24, 0, 94, 100});
        DrawTexturePro(navball_texture->texture, navball_source_rec, navball_dest_rec, (Vector2){0,0}, 0.0f, WHITE);

        GuiDrawText("Press T for tools",(Rectangle){20,10,700,50},TEXT_ALIGN_LEFT,((Color){226, 238, 225,255}));
        GuiDrawText(TextFormat("Time Warp: %iX",*warp),(Rectangle){20,40,700,50},TEXT_ALIGN_LEFT,((Color){226, 238, 225,255}));
        GuiTextBox((Rectangle){ main_deck.x, main_deck.y, 150, 50 }, TextFormat("Vt: %.2lf",calculateTangentialVelocity(ship->velocity,ship->position)), 128, false);
        GuiTextBox((Rectangle){ main_deck.x+main_deck.width-150, main_deck.y, 150, 50 }, TextFormat("Vr: %.2lf",calculateRadialVelocity(ship->velocity,ship->position)), 128, false);
        GuiDrawText(TextFormat("Apoapsis: %.2lf",ship->apoapsis),(Rectangle){20,GetScreenHeight()-125,700,100},TEXT_ALIGN_LEFT,((Color){226, 238, 225,255}));
        GuiDrawText(TextFormat("Periapsis: %.2lf",ship->periapsis),(Rectangle){20,GetScreenHeight()-165,700,100},TEXT_ALIGN_LEFT,((Color){226, 238, 225,255}));
        GuiDrawText(TextFormat("Inclination: %.2lf",calculateOrbibtalInclination(ship->velocity,ship->position)),(Rectangle){GetScreenWidth()-230,GetScreenHeight()-125,700,100},TEXT_ALIGN_LEFT,((Color){226, 238, 225,255}));
        GuiDrawText(TextFormat("Current Height: %.2lf",vec_3Magnitude(ship->position) - MOON_RADIUS),(Rectangle){GetScreenWidth()-300,GetScreenHeight()-165,700,100},TEXT_ALIGN_LEFT,((Color){226, 238, 225,255}));

        float delta_v_display = (float) ship->delta_v;
        GuiProgressBar((Rectangle){100,GetScreenHeight()-40,300,20},"Delta V",TextFormat("%.2f",delta_v_display),&delta_v_display,0,calculateDeltaV(ISP,STARTING_WET_MASS,DRY_MASS));
        GuiProgressBar((Rectangle){GetScreenWidth()-410,GetScreenHeight()-40,300,20},TextFormat("%.2f",ship->engine_throttle),"Throttle",&ship->engine_throttle,0.0f,100.0f);

        static bool is_window_open = false;
        if (IsKeyPressed(KEY_T)) {
            is_window_open = true;
        }
        if (is_window_open) {
            is_window_open = initToolWindow(ship);
        }  

    EndMode2D();
}

static void simulatePhysics(double dt, Vessel *ship, Terrain *terrain, CelestialBody *body) {
    calculateOrbitalVelocity(dt,ship,terrain,body);
    calculateInertia(ship->wet_mass,ship->depth,ship->width,ship->height);
    updateRotation(dt,ship);
    updateOrbitalElements(ship);
    ship->display_pos = (Vector3) {(float) ship->position.x / RENDER_SCALE, (float) ship->position.y / RENDER_SCALE, (float) ship->position.z / RENDER_SCALE};
    if (ship->apoapsis >= 59490000.0+1) {
        *ship = resetShip(*ship);
    }
}

static void render(GameInstance *game, RenderTexture2D *navball_texture, int* warp) {
    BeginDrawing();
        ClearBackground(BLACK);
        drawWorld(game->models ,&game->lander ,&game->the_moon ,&game->world_camera ,&game->engine_particles);
        initalizeUI(game->textures ,game->models ,&game->ui_camera ,&game->navball_camera ,&game->lander ,&game->the_moon ,navball_texture,warp);
    EndDrawing();
}

static void gameLoop(GameInstance *game) {
    int time_warp = 1;
    RenderTexture2D navball_texture = LoadRenderTexture(200, 200);

    while (!WindowShouldClose()) {
        double dt = GetFrameTime() * time_warp;

        if (IsKeyPressed(KEY_LEFT_BRACKET) && time_warp > 0) {
            time_warp -= 1;
        }
        
        if (IsKeyPressed(KEY_RIGHT_BRACKET)&& time_warp < 4) {
            time_warp += 1;
        }

        simulateEngineParticles(&game->lander, &game->engine_particles);

        simulatePhysics(dt, &game->lander, &game->lunar_terrain,&game->the_moon);

        cameraMovement(&game->world_camera ,&game->the_moon ,&game->lander, &game->lunar_terrain);

        throttleControl(&game->lander);

        render(game,&navball_texture,&time_warp);
    }
}

static Vessel initShip(Model *models[MAX_NUM_OF_MODELS]) {

    double starting_apoapsis = 2000;
    double starting_periapsis = 2000;
    vec_3 starting_position;
    vec_3 starting_velocity = calculateStartingVelocity(&starting_position,starting_apoapsis,starting_periapsis,false);
    Vector3 starting_display_pos = { starting_position.x/RENDER_SCALE, starting_position.y/RENDER_SCALE, starting_position.z/RENDER_SCALE };
    vec_3 starting_accelration = calculateOrbitalAcceleration(starting_position);
    float width = 4.3;
    float depth = 4.3;
    float height = 4.2;
    Quaternion starting_rotation = QuaternionIdentity();
    Vector3 ship_inertia = calculateInertia(STARTING_WET_MASS, depth, width, height); 

    return (Vessel) {
        *models[SHIP_MODEL], 
        starting_position, 
        starting_display_pos, 

        width, 
        depth, 
        height,
                        
        starting_rotation,
        (Vector3){0.0, 0.0, 0.0},
        ship_inertia.x,
        ship_inertia.y,
        ship_inertia.z,
                        
        DRY_MASS,
        STARTING_WET_MASS,
        ISP,
        calculateExhaustVelocity(ISP),
        calculateDeltaV(ISP,STARTING_WET_MASS,DRY_MASS),
        10.2,
        0.0f,   

        starting_apoapsis,
        starting_periapsis,
        starting_velocity,
        starting_accelration,

        false,
        false,
        false
    };
}

static CelestialBody initMoon(Model* models[MAX_NUM_OF_MODELS]) {
    vec_3 moon_starting_pos = { 0.0, 0.0, 0.0};
    Vector3 moon_starting_display_pos = { 0.0f, 0.0f, 0.0f};

    return (CelestialBody) {
        *models[MOON_MODEL],
        moon_starting_pos,
        moon_starting_display_pos,
    };
}

static Terrain initTerrain(Model* models[MAX_NUM_OF_MODELS]) {
    return (Terrain) {
        *models[TERRAIN_MODEL],
        (Vector3) {0, 0, 0},
        GetModelBoundingBox(*models[TERRAIN_MODEL])
    };
}

static ParticleSystem initEngineParticles(Matrix *transforms) {
    Shader shader = LoadShader("shaders/instancing.vs", "shaders/color.fs");
    Material matInstances = LoadMaterialDefault();
    matInstances.shader = shader;
    Mesh cube = GenMeshCube(0.07f,0.07f,0.07f);
    return (ParticleSystem){
        cube,
        shader,
        matInstances,
        transforms
    };
}

static Camera3D initWorldCamera(Vessel *ship, CelestialBody *body) {
    Vector3 localUp = Vector3Normalize(Vector3Subtract(ship->display_pos, body->display_pos));
    Vector3 localRight   = Vector3Normalize(Vector3CrossProduct(Y_HAT, localUp));
    Vector3 localForward = Vector3CrossProduct(localUp, localRight);

    Camera3D world_camera = {0};
    world_camera.fovy = 60;
    world_camera.projection = CAMERA_PERSPECTIVE;
    world_camera.up = localUp;
    world_camera.target = ship->display_pos;

    return world_camera;
}

static Camera3D initNavballCamera() {
    Camera3D navball_cam = { 0 };
    navball_cam.position = (Vector3){ 0.0f, 0.0f, 3.2f }; 
    navball_cam.target = (Vector3) {0.0f, 0.0f, 0.0f,};  
    navball_cam.up = (Vector3){ 0.0f, 1.0f, 0.0f };       
    navball_cam.fovy = 45.0f;                             
    navball_cam.projection = CAMERA_PERSPECTIVE;

    return navball_cam;
}

static void initGameInstance(GameInstance *game) {

    SetConfigFlags(FLAG_MSAA_4X_HINT); 
    InitWindow(1280, 720, "Moon Landing Sim");
    rlEnableDepthTest();
    rlSetClipPlanes(0.005,1000000);
    SetTargetFPS(120);
    GuiLoadStyleTurbo();
    GuiSetStyle(DEFAULT, TEXT_SIZE, 32);

    loadTextures(game->textures);
    loadModels(game->models);

    game->particle_transforms = (Matrix *)RL_CALLOC(MAX_PARTICLES, sizeof(Matrix));
    game->engine_particles = initEngineParticles(game->particle_transforms);

    game->lander = initShip(game->models);
    game->the_moon = initMoon(game->models);
    game->lunar_terrain = initTerrain(game->models);

    game->world_camera = initWorldCamera(&game->lander,&game->the_moon);
    game->navball_camera = initNavballCamera();

    game->ui_camera = (Camera2D) { 0 };
    game->ui_camera.zoom = 1.0f;

    gameLoop(game);
}

int main(int argc, char const *argv[]) {
    GameInstance game;
    initGameInstance(&game);
    return 0;
}
