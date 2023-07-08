


typedef struct init_sensors_s {
    float Kacc, Kgyro, Kmag;
    bool baro;
    float eulerGamma, rateGamma, zGamma;
} init_sensors_t;

typedef struct init_flags_s {
    
    bool verbose, sensors, escarm, UDP, Ibus;
    int mode, control;
} init_flags_t;

typedef struct sensor_s {
    float roll, pitch, yaw;
    float rollrate, pitchrate, yawrate;
    float estimatedZ, velocityZ, groundZ;
} sensor_t;

typedef struct controller_s {
    float fx;
    float fy;
    float fz;
    float absz;
    float tx;
    float ty;
    float tz;
    bool ready;
} controller_t;

typedef struct actuation_s {
    float m1;
    float m2;
    float s1;
    float s2;
} actuation_t;


typedef struct feedback_s {
    bool roll, pitch, yaw, x, y, z, rotation;
    float Croll, Cpitch, Cyaw, Cx, Cy, Cz, Cabsz;
    float kproll, kdroll, kppitch, kdpitch, kpyaw, kdyaw;
    float kpx, kdx, kpy, kdy, kpz, kdz;
    float lx;
} feedback_t;