#include <math.h>
#include "robot_body_trajectory.h"
#include "robot_gait_profile.h"

#define N ROBOT_BODY_TRAJECTORY_SAMPLES
#define GRAVITY_MM_S2 9810.0f

static robot_planar_point_t add(robot_planar_point_t a, robot_planar_point_t b)
{ return (robot_planar_point_t){a.x + b.x, a.z + b.z}; }
static robot_planar_point_t sub(robot_planar_point_t a, robot_planar_point_t b)
{ return (robot_planar_point_t){a.x - b.x, a.z - b.z}; }
static robot_planar_point_t mul(robot_planar_point_t a, float s)
{ return (robot_planar_point_t){a.x * s, a.z * s}; }
static float dot(robot_planar_point_t a, robot_planar_point_t b)
{ return a.x * b.x + a.z * b.z; }
static float cross(robot_planar_point_t a, robot_planar_point_t b)
{ return a.x * b.z - a.z * b.x; }

// SE(2) exponential; sinc/cosc series avoid a turn threshold and cancellation
// near zero yaw. Murray/Li/Sastry, A Mathematical Introduction to Robotic
// Manipulation, rigid-body exponential coordinates (restricted to the plane).
robot_vec3_t robot_body_twist_transform(robot_vec3_t p, robot_body_twist_t v, float t)
{
    const float a = v.yaw_rad_s * t;
    const float a2 = a*a;
    const float sinc = fabsf(a) < 0.01f ? 1-a2/6+a2*a2/120 : sinf(a)/a;
    const float cosc = fabsf(a) < 0.01f ? a*(0.5f-a2/24+a2*a2/720) : 2*sinf(a/2)*sinf(a/2)/a;
    const float c = cosf(a), s = sinf(a);
    return (robot_vec3_t){c*p.x-s*p.z+t*(sinc*v.x_mm_s-cosc*v.z_mm_s), p.y,
                         s*p.x+c*p.z+t*(cosc*v.x_mm_s+sinc*v.z_mm_s)};
}

robot_body_twist_t robot_body_trajectory_twist(const robot_body_trajectory_request_t *r)
{
    if (!r || r->stepping_in_place) return (robot_body_twist_t){0};
    const float duty = robot_locomotion_duty_factor(&r->profile);
    if (duty <= 0) return (robot_body_twist_t){0};
    const float speed = r->stride_mm * robot_locomotion_frequency_hz(&r->profile) / duty;
    // 220 mm is the command's yaw lever arm, not a separate radius per foot.
    return (robot_body_twist_t){r->spin ? 0 : speed*r->forward,
                               r->spin ? 0 : speed*r->lateral, speed*r->turn/220.0f};
}

robot_vec3_t robot_body_trajectory_foot(const robot_body_trajectory_request_t *r,
                                       robot_leg_t leg, float phase)
{
    robot_vec3_t base = robot_kinematics_neutral_foot(&r->geometry, leg);
    base.z += leg == ROBOT_LEG_LEFT_FRONT || leg == ROBOT_LEG_LEFT_REAR
        ? r->lateral_stance_mm : -r->lateral_stance_mm;
    if (r->gait == 5 && leg == r->excluded_leg) {
        base.y = fmaxf(40, r->profile.step_height_mm);
        return base;
    }
    const robot_locomotion_leg_phase_t state = r->gait == 5
        ? robot_locomotion_tripod_phase(leg, r->excluded_leg, phase, &r->profile)
        : robot_locomotion_leg_phase(r->gait, leg, phase, &r->profile);
    const float period = 1 / robot_locomotion_frequency_hz(&r->profile);
    const float duty = robot_locomotion_duty_factor(&r->profile);
    const float stance_time = duty*period, swing_time = (1-duty)*period;
    const robot_body_twist_t twist = robot_body_trajectory_twist(r);
    if (state.support_contact)
        // One rigid inverse body motion under stationary contacts. Relative
        // distances of any two simultaneous supports are exactly invariant.
        return robot_body_twist_transform(base, twist, stance_time/2-state.local_phase*period);
    const robot_vec3_t liftoff = robot_body_twist_transform(base, twist, -stance_time/2);
    const robot_vec3_t touchdown = robot_body_twist_transform(base, twist, swing_time+stance_time/2);
    // MIT FootSwingTrajectory is defined in the inertial frame, including
    // zero endpoint velocity. Transform the entire curve back, not its scalar
    // stroke onto four different circular arcs. This is C1 at both contacts.
    const robot_vec3_t world = robot_locomotion_swing_bezier(liftoff, touchdown,
        state.swing_phase, r->profile.step_height_mm);
    return robot_body_twist_transform(world, twist, -swing_time*state.swing_phase);
}

bool robot_body_support_reference(const robot_planar_point_t feet[4],
                                   const bool contact[4], robot_planar_point_t reference,
                                   robot_planar_point_t *out)
{
    if (!feet || !contact || !out || !isfinite(reference.x) || !isfinite(reference.z)) return false;
    const int order[4] = {0, 1, 3, 2};
    robot_planar_point_t hull[4];
    int count = 0;
    for (int j = 0; j < 4; ++j) if (contact[order[j]]) {
        if (!isfinite(feet[order[j]].x) || !isfinite(feet[order[j]].z)) return false;
        hull[count++] = feet[order[j]];
    }
    if (!count) return false; // no constant-height support model in flight
    if (count == 1) { *out = hull[0]; return true; }
    float best_distance = INFINITY;
    bool positive = false, negative = false;
    for (int j = 0; j < count; ++j) {
        const robot_planar_point_t a = hull[j], edge = sub(hull[(j + 1) % count], a);
        const robot_planar_point_t relative = sub(reference, a);
        const float side = cross(edge, relative);
        positive |= side > 0.0001f; negative |= side < -0.0001f;
        const float length2 = dot(edge, edge);
        const float t = length2 > 0.0001f ? fminf(1, fmaxf(0, dot(relative, edge) / length2)) : 0;
        const robot_planar_point_t candidate = add(a, mul(edge, t));
        const robot_planar_point_t error = sub(reference, candidate);
        const float distance = dot(error, error);
        if (distance < best_distance) { best_distance = distance; *out = candidate; }
    }
    if (count > 2 && !(positive && negative)) *out = reference;
    return true;
}

// Complex arithmetic is a compact 2-D rotation operator, not a second
// physical model. In a frame with constant twist (v,w):
//   xi'  = (lambda - i*w) xi  - lambda*p - v
//   eta' = (-lambda - i*w) eta + lambda*p - v
// where xi=c+u/lambda, eta=c-u/lambda, u is WORLD CoM velocity
// expressed in that frame. This includes transport/centripetal terms during
// turns, which a straight-line LIPM applied to rotating coordinates omits.
static robot_planar_point_t cmul(robot_planar_point_t a, robot_planar_point_t b)
{ return (robot_planar_point_t){a.x*b.x-a.z*b.z, a.x*b.z+a.z*b.x}; }
static robot_planar_point_t cinverse(robot_planar_point_t a)
{ return mul((robot_planar_point_t){a.x,-a.z}, 1/dot(a,a)); }
static robot_planar_point_t jrotate(robot_planar_point_t a)
{ return (robot_planar_point_t){-a.z,a.x}; }
typedef struct { robot_planar_point_t decay, constant, ramp; } response_t;

static response_t response(float lambda, float yaw, float dt, bool backward)
{
    const robot_planar_point_t k = {lambda, backward ? -yaw : yaw};
    const robot_planar_point_t z = mul(k,dt);
    const float radius = expf(-z.x);
    const robot_planar_point_t decay = {radius*cosf(z.z), -radius*sinf(z.z)};
    robot_planar_point_t constant, ramp;
    if (dot(z,z) < 0.01f) {
        // Integrals of exp(-k*t), evaluated by series near dt=0.
        const robot_planar_point_t z2=cmul(z,z), z3=cmul(z2,z), z4=cmul(z3,z);
        constant=mul(add(add((robot_planar_point_t){1,0},mul(z,-0.5f)),
                     add(mul(z2,1.0f/6),add(mul(z3,-1.0f/24),mul(z4,1.0f/120)))),dt);
        ramp=mul(add(add((robot_planar_point_t){0.5f,0},mul(z,-1.0f/3)),
                 add(mul(z2,1.0f/8),add(mul(z3,-1.0f/30),mul(z4,1.0f/144)))),dt);
    } else {
        const robot_planar_point_t inv=cinverse(k);
        constant=cmul(sub((robot_planar_point_t){1,0},decay),inv);
        ramp=cmul(sub(constant,mul(decay,dt)),mul(inv,1/dt));
    }
    if (!backward) ramp=sub(constant,ramp);
    return (response_t){decay,constant,ramp};
}

static robot_planar_point_t propagate(robot_planar_point_t state,
    robot_planar_point_t p0, robot_planar_point_t p1, response_t kernel,
    float lambda, robot_planar_point_t velocity, bool backward)
{
    const robot_planar_point_t force=add(mul(p0,lambda),mul(velocity,backward ? 1 : -1));
    return add(cmul(kernel.decay,state), add(cmul(kernel.constant,force),
                cmul(kernel.ramp,mul(sub(p1,p0),lambda))));
}

static bool solve_twist(robot_body_trajectory_t *p,
    const robot_planar_point_t support[N], float height, float period, robot_body_twist_t twist)
{
    if (!p) return false;
    p->valid=false;
    if (!support || !isfinite(height) || height<50 || height>500 ||
        !isfinite(period) || period<0.25f || period>10 ||
        !isfinite(twist.x_mm_s) || !isfinite(twist.z_mm_s) || !isfinite(twist.yaw_rad_s)) return false;
    for (int i=0;i<N;++i) {
        if (!isfinite(support[i].x) || !isfinite(support[i].z)) return false;
        p->support[i]=support[i];
    }
    p->twist=twist;
    p->effective_frequency_hz=1/period;
    p->joint_path_feasible=true;
    p->omega=sqrtf(GRAVITY_MM_S2/height); p->dt=period/N;
    const robot_planar_point_t velocity={twist.x_mm_s,twist.z_mm_s};
    const response_t back=response(p->omega,twist.yaw_rad_s,p->dt,true);
    const response_t forward=response(p->omega,twist.yaw_rad_s,p->dt,false);
    const robot_planar_point_t back_gain=cinverse(sub((robot_planar_point_t){1,0},
        response(p->omega,twist.yaw_rad_s,period,true).decay));
    const robot_planar_point_t forward_gain=cinverse(sub((robot_planar_point_t){1,0},
        response(p->omega,twist.yaw_rad_s,period,false).decay));
    robot_planar_point_t xi={0},eta={0};
    for (int i=N-1;i>=0;--i)
        xi=propagate(xi,p->support[i],p->support[(i+1)%N],back,p->omega,velocity,true);
    xi=cmul(xi,back_gain);
    for (int i=N-1;i>=0;--i) {
        xi=propagate(xi,p->support[i],p->support[(i+1)%N],back,p->omega,velocity,true);
        p->divergent[i]=xi;
    }
    for (int i=0;i<N;++i)
        eta=propagate(eta,p->support[i],p->support[(i+1)%N],forward,p->omega,velocity,false);
    eta=cmul(eta,forward_gain);
    for (int i=0;i<N;++i) {
        p->convergent[i]=eta;
        eta=propagate(eta,p->support[i],p->support[(i+1)%N],forward,p->omega,velocity,false);
    }
    p->valid=true;
    return true;
}

bool robot_body_trajectory_solve(robot_body_trajectory_t *p,
    const robot_planar_point_t support[N], float height, float period)
{
    return solve_twist(p,support,height,period,(robot_body_twist_t){0});
}

static bool same_request(const robot_body_trajectory_request_t *a,
                         const robot_body_trajectory_request_t *b)
{
    return a->gait == b->gait && a->profile.stride_mm == b->profile.stride_mm &&
        a->profile.step_height_mm == b->profile.step_height_mm &&
        a->profile.frequency_centi_hz == b->profile.frequency_centi_hz &&
        a->profile.duty_percent == b->profile.duty_percent &&
        a->geometry.frame_length_mm == b->geometry.frame_length_mm &&
        a->geometry.frame_width_mm == b->geometry.frame_width_mm &&
        a->geometry.corner_inset_mm == b->geometry.corner_inset_mm &&
        a->geometry.abduction_link_mm == b->geometry.abduction_link_mm &&
        a->geometry.upper_leg_mm == b->geometry.upper_leg_mm &&
        a->geometry.lower_leg_mm == b->geometry.lower_leg_mm &&
        a->com_height_mm == b->com_height_mm && a->lateral_stance_mm == b->lateral_stance_mm &&
        a->forward == b->forward && a->lateral == b->lateral && a->turn == b->turn &&
        a->stride_mm == b->stride_mm && a->stepping_in_place == b->stepping_in_place &&
        a->spin == b->spin && a->excluded_leg == b->excluded_leg &&
        a->body_height_mm == b->body_height_mm && a->com_offset_x_mm == b->com_offset_x_mm &&
        a->com_offset_z_mm == b->com_offset_z_mm && a->joint_velocity_limit == b->joint_velocity_limit &&
        a->joint_acceleration_limit == b->joint_acceleration_limit;
}

// Uniform path time-scaling: velocity scales with f, acceleration with f².
// Differentiate the FULL IK path including the planned torso, so slowing one
// knee never silently gives it a phase different from the other eleven axes.
static float joint_path_scale(const robot_body_trajectory_t *plan,
                              const robot_body_trajectory_request_t *r)
{
    float prev[4][3]={{0}}, prev2[4][3]={{0}}, vmax=0, amax=0;
    for (int i=0;i<N+2;++i) {
        const robot_planar_point_t body=mul(add(plan->divergent[i%N],plan->convergent[i%N]),0.5f);
        for (int leg=0;leg<4;++leg) {
            robot_vec3_t foot=plan->feet[i%N][leg];
            foot.x-=body.x-r->com_offset_x_mm;
            foot.z-=body.z-r->com_offset_z_mm;
            float q[3];
            if (!robot_kinematics_inverse(&r->geometry,leg,foot,r->body_height_mm,q)) return 0;
            for (int axis=0;axis<3;++axis) {
                if (i>0) vmax=fmaxf(vmax,fabsf(q[axis]-prev[leg][axis])/plan->dt);
                if (i>1) amax=fmaxf(amax,fabsf(q[axis]-2*prev[leg][axis]+prev2[leg][axis])/(plan->dt*plan->dt));
                prev2[leg][axis]=prev[leg][axis]; prev[leg][axis]=q[axis];
            }
        }
    }
    return fminf(r->joint_velocity_limit/fmaxf(vmax,0.0001f),
                 sqrtf(r->joint_acceleration_limit/fmaxf(amax,0.0001f)));
}

bool robot_body_trajectory_matches(const robot_body_trajectory_t *p,
                                   const robot_body_trajectory_request_t *r)
{
    return p && r && p->request_cached && same_request(&p->request, r);
}

bool robot_body_trajectory_build(robot_body_trajectory_t *p,
                                 const robot_body_trajectory_request_t *r)
{
    if (!p || !r || (r->gait == 5 && r->excluded_leg >= ROBOT_LEG_COUNT)) return false;
    if (p->request_cached && same_request(&p->request, r)) return p->valid;
    p->request = *r; p->request_cached = true; p->valid = false;
    if (!robot_locomotion_profile_valid(r->gait, &r->profile) ||
        // Below 50% duty diagonal gaits have unsupported flight phases.
        (!robot_locomotion_is_single_support_gait(r->gait) && r->profile.duty_percent < 50))
        return false;
    for (int i = 0; i < N; ++i) {
        const float phase = (float)i / N;
        robot_planar_point_t feet[4]; float availability[4]; bool contact[4];
        for (int leg = 0; leg < 4; ++leg) {
            const robot_vec3_t foot = robot_body_trajectory_foot(r, (robot_leg_t)leg, phase);
            p->feet[i][leg]=foot;
            const robot_locomotion_leg_phase_t state = r->gait == 5
                ? robot_locomotion_tripod_phase((robot_leg_t)leg, r->excluded_leg, phase, &r->profile)
                : robot_locomotion_leg_phase(r->gait, (robot_leg_t)leg, phase, &r->profile);
            feet[leg] = (robot_planar_point_t){foot.x, foot.z};
            contact[leg] = state.support_contact;
            availability[leg] = robot_predictive_support_availability(state.local_phase,
                robot_locomotion_duty_factor(&r->profile));
        }
        robot_planar_point_t vpsp;
        if (r->gait == 5) {
            // The quadruped VPSP's neighbour-pair formula assumes four legs.
            // For three legs use the centroid of scheduled support as ZMP,
            // then the same periodic LIPM preview; never count the held leg.
            vpsp = (robot_planar_point_t){0};
            unsigned n = 0;
            for (int leg=0; leg<4; ++leg) if (contact[leg]) {
                vpsp = add(vpsp, feet[leg]); ++n;
            }
            if (n < 2) return false;
            vpsp = mul(vpsp, 1.0f/n);
        } else if (!robot_predictive_support_target(feet, availability, &vpsp)) return false;
        if (!robot_body_support_reference(feet, contact, vpsp, &p->support[i])) return false;
    }
    robot_body_trajectory_request_t timed=*r;
    for (int iteration=0;iteration<6;++iteration) {
        if (!solve_twist(p,p->support,r->com_height_mm,
                1/robot_locomotion_frequency_hz(&timed.profile),robot_body_trajectory_twist(&timed))) return false;
        if (r->joint_velocity_limit<=0 || r->joint_acceleration_limit<=0) return true;
        const float scale=joint_path_scale(p,r);
        if (scale>=1) return true;
        if (scale<=0 || timed.profile.frequency_centi_hz<=15) break;
        // Numerical reserve for the sampled derivative. Keep the request in
        // NVS; only the runtime phase rate and its LIPM period change together.
        timed.profile.frequency_centi_hz=(uint16_t)fmaxf(15,
            floorf(timed.profile.frequency_centi_hz*fminf(0.95f,scale*0.95f)));
    }
    p->joint_path_feasible=false;
    return p->valid=false;
}

robot_body_trajectory_sample_t robot_body_trajectory_sample(const robot_body_trajectory_t *p,
                                                            float phase)
{
    if (!p || !p->valid || !isfinite(phase)) return (robot_body_trajectory_sample_t){0};
    phase -= floorf(phase);
    const float index = phase * N;
    const int i = (int)index, next = (i + 1) % N;
    const float u = index - i;
    const robot_planar_point_t support = add(p->support[i], mul(sub(p->support[next], p->support[i]), u));
    const robot_planar_point_t eta = u > 0.000001f
        ? propagate(p->convergent[i], p->support[i], support,
            response(p->omega,p->twist.yaw_rad_s,p->dt*u,false),p->omega,
            (robot_planar_point_t){p->twist.x_mm_s,p->twist.z_mm_s},false)
        : p->convergent[i];
    const robot_planar_point_t xi = u < 0.999999f
        ? propagate(p->divergent[next], support, p->support[next],
            response(p->omega,p->twist.yaw_rad_s,p->dt*(1-u),true),p->omega,
            (robot_planar_point_t){p->twist.x_mm_s,p->twist.z_mm_s},true)
        : p->divergent[next];
    const robot_planar_point_t position = mul(add(xi, eta), 0.5f);
    const robot_planar_point_t v={p->twist.x_mm_s,p->twist.z_mm_s};
    const float w=p->twist.yaw_rad_s;
    const robot_planar_point_t velocity=sub(sub(mul(sub(xi,eta),p->omega*0.5f),v),mul(jrotate(position),w));
    const robot_planar_point_t acceleration=sub(add(mul(sub(position,support),p->omega*p->omega),
        mul(position,w*w)),add(mul(jrotate(velocity),2*w),mul(jrotate(v),w)));
    return (robot_body_trajectory_sample_t){
        .position_mm = position,
        .velocity_mm_s = velocity,
        .acceleration_mm_s2 = acceleration,
        .support_mm = support,
    };
}
