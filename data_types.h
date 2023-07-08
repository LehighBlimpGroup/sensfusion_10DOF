


typedef struct sensors_s {
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