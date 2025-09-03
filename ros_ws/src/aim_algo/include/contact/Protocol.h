#pragma once

#include <cstdint>

struct ProjectileRx {
    uint8_t header;  // 0xA3 for InfantryDL
    float roll;
    float pitch;
    float yaw;
    float q[4];
    uint8_t color;
    uint8_t auto_aim_mode;
    uint8_t shoot_decision;
    float bullet_spped;
    uint8_t EOF_;  // 0xAA for InfantryDL // EOF_ not EOF (variable name)
} __attribute__((packed));

struct ProjectileTx {
    // WARNING:: the found, patrolling, done_fitting, is_updated are not clear
    uint8_t header;  // 0xA3 for InfantryDL
    float pitch;
    float yaw;
    uint8_t found;
    uint8_t shoot_or_not;
    uint8_t done_fitting;
    uint8_t patrolling;  // 0xAA for InfantryDL // EOF_ not EOF (variable name)
    uint8_t is_updated;
    uint8_t checksum;
} __attribute__((packed));
