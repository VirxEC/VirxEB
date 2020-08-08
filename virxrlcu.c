#include <math.h>
#include <Python.h>

// Constants

int max_speed = 2300;
double jump_max_duration = 0.2;
double jump_speed = 291 + (2/3);
double jump_acc = 1458 + (1/3);

// Vector stuff

typedef struct vector {
    double x;
    double y;
    double z;
} Vector;

struct opti_norm {
    Vector vector;
    double magnitude;
};

Vector add(Vector vec1, Vector vec2) {
    return (Vector) {
        .x=vec1.x + vec2.x, .y=vec1.y + vec2.y, .z=vec1.z + vec2.z
    };
};

Vector sub(Vector vec1, Vector vec2) {
    return (Vector) {
        .x=vec1.x - vec2.x, .y=vec1.y - vec2.y, .z=vec1.z - vec2.z
    };
};

Vector multiply(Vector vec1, Vector vec2) {
    return (Vector) {
        .x=vec1.x * vec2.x, .y=vec1.y * vec2.y, .z=vec1.z * vec2.z
    };
};

Vector equal_power(Vector vec1, int vec2) {
    double r = pow(vec1.x, vec2);
    return (Vector) {
        .x=r, .y=r, .z=r
    };
};

Vector divide(Vector vec1, Vector vec2) {
    return (Vector) {
        .x=vec1.x / vec2.x, .y=vec1.y / vec2.y, .z=vec1.z / vec2.z
    };
};

double dot(Vector vec1, Vector vec2) {
    return vec1.x * vec2.x + vec1.y * vec2.y + vec1.z * vec2.z;
};

double magnitude(Vector vec) {
    return sqrt(dot(vec, vec));
};

struct opti_norm normalize(Vector vec) {
    struct opti_norm r;
    r.magnitude = magnitude(vec);

    r.vector = (r.magnitude == 0) ? (Vector) {
        0, 0, 0
    } : (Vector) {
            .x=vec.x/r.magnitude, .y=vec.y/r.magnitude, .z=vec.z/r.magnitude
        };

        return r;
};

Vector flatten(Vector vec) {
    return (Vector) {
        .x=vec.x, .y=vec.y, .z=0
    };
}

double angle(Vector vec1, Vector vec2) {
    return acos(max(min(dot(normalize(vec1).vector, normalize(vec2).vector), 1), -1));
};

Vector cross(Vector vec1, Vector vec2) {
    return (Vector) {
        .x=(vec1.y * vec2.z) - (vec1.z * vec2.y), .y=(vec1.z * vec2.x) - (vec1.x * vec2.z), .z=(vec1.x * vec2.y) - (vec1.y * vec2.x)
    };
};

Vector double_to_vector(double num) {
    return (Vector) {
        .x=num, .y=num, .z=num
    };
}

Vector clamp(Vector vec, Vector start, Vector end) {
    Vector s = normalize(vec).vector;
    _Bool right = dot(s, cross(end, (Vector) {
        .x=0, .y=0, .z=-1
    })) < 0;
    _Bool left = dot(s, cross(start, (Vector) {
        .x=0, .y=0, .z=-1
    })) > 0;

    if ((dot(end, cross(start, (Vector) {
        .x=0, .y=0, .z=-1
    })) > 0) ? (right && left) : (right || left)) {
        return vec;
    }

    if (dot(start, s) < dot(end, s))
        return end;

    return start;
}

double distance(Vector vec1, Vector vec2) {
    return magnitude(sub(vec1, vec2));
};

// Main lib

struct jump_shot {
    int found;
    Vector best_shot_vector;
    int direction;
};

struct aerial_shot {
    int found;
    Vector ball_intercept;
};

struct post_correction {
    Vector left;
    Vector right;
    _Bool swapped;
};

struct post_correction correct_for_posts(Vector ball_location, Vector left_target, Vector right_target) {
    double ball_radius = 120;
    Vector goal_line_perp = cross(sub(right_target, left_target), (Vector) {
        .x=0, .y=0, .z=1
    });
    Vector left = add(left_target, cross(normalize(sub(left_target, ball_location)).vector, multiply((Vector) {
        .x=0, .y=0, .z=1
    }, double_to_vector(ball_radius))));
    Vector right = add(right_target, cross(normalize(sub(right_target, ball_location)).vector, multiply((Vector) {
        .x=0, .y=0, .z=1
    }, double_to_vector(ball_radius))));

    struct post_correction r;
    r.left = (dot(sub(left, left_target), goal_line_perp) > 0) ? left_target : left;
    r.right = (dot(sub(right, right_target), goal_line_perp) > 0) ? right_target : right;
    r.swapped = dot(cross(normalize(sub(left, ball_location)).vector, (Vector) {
        .x=0, .y=0, .z=1
    }), normalize(sub(right, ball_location)).vector) > -0.1;

    return r;
};

_Bool in_field(Vector point, int radius) {
    point = (Vector){ .x=fabs(point.x), .y=fabs(point.y), .z=fabs(point.z) };
    return !((point.x > 4080 - radius) || (point.y > 5900 - radius) || (point.x > 880 - radius && point.y > 5105 - radius) || (point.x > 2650 && point.y > -point.x + 8025 - radius));
};

_Bool fast_shot(Vector car_location, Vector ball_intercept, double time_remaining, double cap_, int speed) {
    return time_remaining <= min(cap_, distance(car_location, ball_intercept) / speed);
}

double find_slope(Vector shot_vector, Vector car_to_target) {
    double d = dot(shot_vector, car_to_target);
    double e = fabs(dot(cross(shot_vector, (Vector) {
        .x=0, .y=0, .z=1
    }), car_to_target));

    if (e == 0)
        return 10 * copysign(1, d);

    return max(min(d / e, 3), -3);
};

double cap(value, min, max) {
    if (value < min)
        return min;
    if (value > max)
        return max;
    return value;
};

struct jump_shot parse_slice_for_jump_shot_with_target(double time_remaining, Vector ball_location, Vector car_location, Vector car_forward, int car_boost, Vector left_target, Vector right_target, double cap_) {
    struct jump_shot r;
    r.found = -1;

    Vector car_to_ball = sub(ball_location, car_location);
    struct opti_norm car_to_ball_norm = normalize(car_to_ball);
    Vector direction = car_to_ball_norm.vector;
    double distance = car_to_ball_norm.magnitude;

    double forward_angle = angle(flatten(direction), flatten(car_forward));
    double backward_angle = Py_MATH_PI - forward_angle;

    double forward_time = time_remaining - (forward_angle * 0.318);
    double backward_time = time_remaining - (backward_angle * 0.418);

    double forward_flag = forward_time > 0 && (distance*1.05 / forward_time) < ((car_boost > distance/100) ? 2275 : 1400);
    double backward_flag = distance < 1500 && backward_time > 0 && (distance*1.05 / backward_time) < 1200;

    if (forward_flag || backward_flag) {
        struct post_correction pst_crrctn = correct_for_posts(ball_location, left_target, right_target);
        if (!pst_crrctn.swapped) {
            Vector left_vector = normalize(sub(pst_crrctn.left, ball_location)).vector;
            Vector right_vector = normalize(sub(pst_crrctn.right, ball_location)).vector;
            r.best_shot_vector = clamp(direction, left_vector, right_vector);

            if (in_field(sub(ball_location, multiply(r.best_shot_vector, double_to_vector(200))), 1) && fast_shot(car_location, ball_location, time_remaining, cap_, 250)) {
                double slope = find_slope(r.best_shot_vector, car_to_ball);
                if (forward_flag && ball_location.z <= 275 && slope > 0) {
                    r.found = 1;
                    r.direction = 1;
                    return r;
                }
                else if (backward_flag && ball_location.z < 250 && slope > 1.5 && time_remaining < cap_/2) {
                    r.found = 1;
                    r.direction = -1;
                    return r;
                }
            }
        }
    }

    return r;
};

struct jump_shot parse_slice_for_jump_shot(double time_remaining, Vector ball_location, Vector car_location, Vector car_forward, int car_boost, double cap_) {
    struct jump_shot r;
    r.found = -1;

    Vector car_to_ball = sub(ball_location, car_location);
    struct opti_norm car_to_ball_norm = normalize(car_to_ball);
    Vector direction = car_to_ball_norm.vector;
    double distance = car_to_ball_norm.magnitude;

    double forward_angle = angle(flatten(direction), flatten(car_forward));
    double backward_angle = Py_MATH_PI - forward_angle;

    double forward_time = time_remaining - (forward_angle * 0.318);
    double backward_time = time_remaining - (backward_angle * 0.418);

    double forward_flag = forward_time > 0 && (distance / forward_time) < ((car_boost > distance/100) ? 2275 : 1400);
    double backward_flag = distance < 1500 && backward_time > 0 && (distance*1.05 / backward_time) < 1200;

    r.best_shot_vector = direction;

    if ((forward_flag || backward_flag) && in_field(sub(ball_location, multiply(r.best_shot_vector, double_to_vector(200))), 1)) {
        double slope = find_slope(r.best_shot_vector, car_to_ball);
        if (forward_flag && ball_location.z <= 275 && slope > 0) {
            r.found = 1;
            r.direction = 1;
        }
        else if (backward_flag && ball_location.z < 250 && slope > 1.5 && time_remaining < cap_/2) {
            r.found = 1;
            r.direction = -1;
        }
    }

    return r;
};

struct car {
    Vector location;
    Vector velocity;
    Vector up;
    Vector forward;
    int airborne;
    int boost;
};

_Bool is_viable(double time_remaining, double boost_accel, Vector gravity, struct car me, Vector target) {
    Vector T = double_to_vector(time_remaining);
    Vector xf = add(add(me.location, multiply(me.velocity, T)), multiply(divide(gravity, double_to_vector(2)), equal_power(T, 2)));
    Vector vf = add(me.velocity, multiply(gravity, T));

    if (me.airborne == -1) {
        vf = add(vf, multiply(me.up, double_to_vector(2 * jump_speed + jump_acc * jump_max_duration)));
        xf = add(xf, add(multiply(me.up, double_to_vector(jump_speed *  (2 * time_remaining - jump_max_duration))), double_to_vector(jump_acc * (time_remaining * jump_max_duration - pow(jump_max_duration, 2) / 2))));
    }

    Vector delta_x = sub(target, xf);
    Vector f = normalize(delta_x).vector;

    double phi = angle(f, me.forward);
    double turn_time = 0.7 * (2 * sqrt(phi / 9));
    double tau1 = turn_time * cap(1 - 0.3 / phi, 0, 1);
    double required_acc = (2 * magnitude(delta_x) / (pow(time_remaining - tau1, 2)));
    double ratio = required_acc / boost_accel;
    double tau2 = time_remaining - (time_remaining - tau1) * sqrt(1 - cap(ratio, 0, 1));
    double boost_estimate = (tau2 - tau1) * 30;

    Vector velocity_estimate = add(vf, multiply(f, double_to_vector(boost_accel * (tau2 - tau1))));

    _Bool enough_boost = boost_estimate < 0.95 * me.boost;
    _Bool enough_time = fabs(ratio) < 0.9;
    _Bool enough_speed = magnitude(velocity_estimate) < 0.9 * max_speed;

    return enough_speed && enough_boost && enough_time;
};

struct aerial_shot parse_slice_for_aerial_shot_with_target(double time_remaining, double boost_accel, Vector gravity, Vector ball_location, struct car me, Vector left_target, Vector right_target, double cap_) {
    struct aerial_shot r;
    r.found = -1;

    Vector car_to_ball = sub(ball_location, me.location);
    Vector direction = normalize(car_to_ball).vector;

    struct post_correction pst_crrctn = correct_for_posts(ball_location, left_target, right_target);
    if (!pst_crrctn.swapped) {
        Vector left_vector = normalize(sub(pst_crrctn.left, ball_location)).vector;
        Vector right_vector = normalize(sub(pst_crrctn.right, ball_location)).vector;
        Vector best_shot_vector = clamp(direction, left_vector, right_vector);

        if (in_field(sub(ball_location, multiply(best_shot_vector, double_to_vector(200))), 1) && fast_shot(me.location, ball_location, time_remaining, cap_, 250)) {
            r.ball_intercept = sub(ball_location, multiply(best_shot_vector, double_to_vector(92)));
            double slope = find_slope(r.ball_intercept, car_to_ball);

            if (slope > -1 && is_viable(time_remaining, boost_accel, gravity, me, r.ball_intercept))
                r.found = 1;
        }
    }

    return r;
};

struct aerial_shot parse_slice_for_aerial_shot(double time_remaining, double boost_accel, Vector gravity, Vector ball_location, struct car me, double cap_) {
    struct aerial_shot r;
    r.found = -1;

    Vector car_to_ball = sub(ball_location, me.location);
    Vector best_shot_vector = normalize(car_to_ball).vector;

    if (in_field(sub(ball_location, multiply(best_shot_vector, double_to_vector(200))), 1)) {
        r.ball_intercept = sub(ball_location, multiply(best_shot_vector, double_to_vector(92)));

        if (is_viable(time_remaining, boost_accel, gravity, me, r.ball_intercept))
            r.found = 1;
    }

    return r;
};

static PyObject *method_parse_slice_for_jump_shot_with_target(PyObject *self, PyObject *args) {
    int car_boost;
    double time_remaining, cap_;
    Vector ball_location, car_location, car_forward, left_target, right_target;

    if (!PyArg_ParseTuple(args, "d(ddd)(ddd)(ddd)i(ddd)(ddd)d", &time_remaining, &ball_location.x, &ball_location.y, &ball_location.z, &car_location.x, &car_location.y, &car_location.z, &car_forward.x, &car_forward.y, &car_forward.z, &car_boost, &left_target.x, &left_target.y, &left_target.z, &right_target.x, &right_target.y, &right_target.z, &cap_))
        return NULL;

    struct jump_shot data_struct = parse_slice_for_jump_shot_with_target(time_remaining, ball_location, car_location, car_forward, car_boost, left_target, right_target, cap_);

    return Py_BuildValue("{s:i,s:(ddd),s:i}", "found", data_struct.found, "best_shot_vector", data_struct.best_shot_vector.x, data_struct.best_shot_vector.y, data_struct.best_shot_vector.z, "direction", data_struct.direction);
};

static PyObject *method_parse_slice_for_jump_shot(PyObject *self, PyObject *args) {
    int car_boost;
    double time_remaining, cap_;
    Vector ball_location, car_location, car_forward;

    if (!PyArg_ParseTuple(args, "d(ddd)(ddd)(ddd)id", &time_remaining, &ball_location.x, &ball_location.y, &ball_location.z, &car_location.x, &car_location.y, &car_location.z, &car_forward.x, &car_forward.y, &car_forward.z, &car_boost, &cap_))
        return NULL;

    struct jump_shot data_struct = parse_slice_for_jump_shot(time_remaining, ball_location, car_location, car_forward, car_boost, cap_);

    return Py_BuildValue("{s:i,s:(ddd),s:i}", "found", data_struct.found, "best_shot_vector", data_struct.best_shot_vector.x, data_struct.best_shot_vector.y, data_struct.best_shot_vector.z, "direction", data_struct.direction);
};

static PyObject *method_parse_slice_for_aerial_shot_with_target(PyObject *self, PyObject *args) {
    struct car me;
    double time_remaining, boost_accel, cap_;
    Vector gravity, ball_location, left_target, right_target;

    if (!PyArg_ParseTuple(args, "dd(ddd)(ddd)((ddd)(ddd)(ddd)(ddd)ii)(ddd)(ddd)d", &time_remaining, &boost_accel, &gravity.x, &gravity.y, &gravity.z, &ball_location.x, &ball_location.y, &ball_location.z, &me.location.x, &me.location.y, &me.location.z, &me.velocity.x, &me.velocity.y, &me.velocity.z, &me.up.x, &me.up.x, &me.up.x, &me.forward.x, &me.forward.x, &me.forward.x, &me.airborne, &me.boost, &left_target.x, &left_target.y, &left_target.z, &right_target.x, &right_target.y, &right_target.z, &cap_))
        return NULL;

    struct aerial_shot data_struct = parse_slice_for_aerial_shot_with_target(time_remaining, boost_accel, gravity, ball_location, me, left_target, right_target, cap_);

    return Py_BuildValue("{s:i,s:(ddd)}", "found", data_struct.found, "ball_intercept", data_struct.ball_intercept.x, data_struct.ball_intercept.y, data_struct.ball_intercept.z);
};

static PyObject *method_parse_slice_for_aerial_shot(PyObject *self, PyObject *args) {
    struct car me;
    double time_remaining, boost_accel, cap_;
    Vector gravity, ball_location;

    if (!PyArg_ParseTuple(args, "dd(ddd)(ddd)((ddd)(ddd)(ddd)(ddd)ii)d", &time_remaining, &boost_accel, &gravity.x, &gravity.y, &gravity.z, &ball_location.x, &ball_location.y, &ball_location.z, &me.location.x, &me.location.y, &me.location.z, &me.velocity.x, &me.velocity.y, &me.velocity.z, &me.up.x, &me.up.x, &me.up.x, &me.forward.x, &me.forward.x, &me.forward.x, &me.airborne, &me.boost, &cap_))
        return NULL;

    struct aerial_shot data_struct = parse_slice_for_aerial_shot(time_remaining, boost_accel, gravity, ball_location, me, cap_);

    return Py_BuildValue("{s:i,s:(ddd)}", "found", data_struct.found, "ball_intercept", data_struct.ball_intercept.x, data_struct.ball_intercept.y, data_struct.ball_intercept.z);
};

static PyMethodDef methods[] ={
    { "parse_slice_for_jump_shot_with_target", method_parse_slice_for_jump_shot_with_target, METH_VARARGS, "Parse slice for a jump shot with a target" },
    { "parse_slice_for_jump_shot", method_parse_slice_for_jump_shot, METH_VARARGS, "Parse slice for a jump shot" },
    { "parse_slice_for_aerial_shot_with_target", method_parse_slice_for_aerial_shot_with_target, METH_VARARGS, "Parse slice for an aerial shot with a target" },
    { "parse_slice_for_aerial_shot", method_parse_slice_for_aerial_shot, METH_VARARGS, "Parse slice for an aerial shot" },
    { NULL, NULL, 0, NULL }
};

static struct PyModuleDef module ={
    PyModuleDef_HEAD_INIT,
    "virxrlcu",
    "C Library for VirxERLU",
    -1,
    methods
};

PyMODINIT_FUNC PyInit_virxrlcu(void) {
    return PyModule_Create(&module);
};