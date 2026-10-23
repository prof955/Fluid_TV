#pragma once
#include <Arduino.h>
#include <math.h>
#include <string.h>
#include "FluidTypes.h"

// A compact 2D adaptation of Clavet's double-density-relaxation particle
// fluid. The ESP32-S3 reference project uses the same position-based idea in
// 3D; this version keeps only 126 particles and a 20x20 spatial hash for the
// classic ESP32 target. Corrected particle positions produce the display mask.
class FluidSimulation {
public:
  FluidSimulation() { clear(); }

  void begin() {
    clear();
    calibrateRestDensity();
    seedPool();
    rebuildGrid();
    computeDensityAndViscosity(0.0f);
    updateBulkMotion();
  }

  void step(const FluidControl &control, float dtSeconds) {
    if (dtSeconds < 0.004f) dtSeconds = 0.004f;
    if (dtSeconds > 0.04f) dtSeconds = 0.04f;
    const float dt = clampf(dtSeconds * TIME_SCALE, 0.003f, SIM_DT_MAX);
    const float dt2 = dt * dt;
    const float damping = expf(-AIR_DRAG * dt);

    downX_ = control.gravityX;
    downY_ = control.gravityY;
    normalize(downX_, downY_);

    // Gravity and the one-shot gyro kick update velocity; the density solver
    // then corrects positions and the corrected displacement becomes velocity.
    for (int i = 0; i < PARTICLE_COUNT; ++i) {
      Particle &p = particles_[i];
      p.vx = (p.vx + GRAVITY_ACCEL * downX_ * dt +
              control.impulseX * IMPULSE_TO_VELOCITY) * damping;
      p.vy = (p.vy + GRAVITY_ACCEL * downY_ * dt +
              control.impulseY * IMPULSE_TO_VELOCITY) * damping;
      clampVelocity(p.vx, p.vy);
      p.oldX = p.x;
      p.oldY = p.y;
      p.x += p.vx * dt;
      p.y += p.vy * dt;
    }

    rebuildGrid();
    computeDensityAndViscosity(dt);
    relaxPositions(dt2);

    const float invDt = 1.0f / dt;
    for (int i = 0; i < PARTICLE_COUNT; ++i) {
      Particle &p = particles_[i];
      p.vx = (p.x - p.oldX) * invDt;
      p.vy = (p.y - p.oldY) * invDt;
      resolveWalls(p);
      clampVelocity(p.vx, p.vy);
    }
    updateBulkMotion();
  }

  float gravityAngleDeg() const {
    return atan2f(downX_, downY_) * RAD_TO_DEG_F;
  }

  float bulkFlowSpeed() const { return bulkFlowSpeed_; }
  uint32_t totalMass() const { return PARTICLE_COUNT; }
  float restDensity() const { return restDensity_; }

void makeSnapshot(FluidSnapshot &snapshot) {
    const float emptyDensity = restDensity_ * DENSITY_EDGE_FRACTION;
    const float fullDensity = restDensity_ + DENSITY_SELF_BIAS;
    const float densityRange = fullDensity - emptyDensity;
    const float invDensityRange = densityRange > 0.0001f ? (1.0f / densityRange) : 0.0f;

    // SOLVER_SCALE == 1 durumu için doğrudan 1:1 hızlı kopyalama (Interpolasyon döngülerini atlar)
    if (FLUID_SOLVER_SCALE == 1) {
      for (int i = 0; i < SOLVER_CELLS; ++i) {
        // Koordinat hesabı yapmak yerine doğrudan hücreden oku
        int cy = i / SOLVER_W;
        int cx = i % SOLVER_W;
        float rho = sampleDensityAt((float)cx + 0.5f, (float)cy + 0.5f);
        float t = (rho - emptyDensity) * invDensityRange;
        t = clampf(t, 0.0f, 1.0f);
        t = t * t * (3.0f - 2.0f * t); // Smoothstep
        snapshot.fill[i] = (uint8_t)(t * 255.0f + 0.5f);
      }
    } else {
      // Orijinal ölçeklemeli kod (Solver 2 vb. için)
      for (int sy = 0; sy < SOLVER_H; ++sy) {
        const float sampleY = ((float)sy + 0.5f) * (float)FLUID_SOLVER_SCALE;
        for (int sx = 0; sx < SOLVER_W; ++sx) {
          const float sampleX = ((float)sx + 0.5f) * (float)FLUID_SOLVER_SCALE;
          float rho = sampleDensityAt(sampleX, sampleY);
          float t = invDensityRange > 0.0f ? (rho - emptyDensity) * invDensityRange : 0.0f;
          t = clampf(t, 0.0f, 1.0f);
          t = t * t * (3.0f - 2.0f * t);
          densityField_[index(sx, sy)] = (uint8_t)(t * 255.0f + 0.5f);
        }
      }

      for (int y = 0; y < FLUID_GRID_H; ++y) {
        const float sy = (float)y / (float)FLUID_SOLVER_SCALE;
        int y0 = (int)sy;
        if (y0 >= SOLVER_H) y0 = SOLVER_H - 1;
        const int y1 = y0 + 1 < SOLVER_H ? y0 + 1 : y0;
        const float ty = sy - (float)y0;
        for (int x = 0; x < FLUID_GRID_W; ++x) {
          const float sx = (float)x / (float)FLUID_SOLVER_SCALE;
          int x0 = (int)sx;
          if (x0 >= SOLVER_W) x0 = SOLVER_W - 1;
          const int x1 = x0 + 1 < SOLVER_W ? x0 + 1 : x0;
          const float tx = sx - (float)x0;
          const float m00 = (float)densityField_[index(x0, y0)];
          const float m10 = (float)densityField_[index(x1, y0)];
          const float m01 = (float)densityField_[index(x0, y1)];
          const float m11 = (float)densityField_[index(x1, y1)];
          const float top = m00 + (m10 - m00) * tx;
          const float bottom = m01 + (m11 - m01) * tx;
          snapshot.fill[y * FLUID_GRID_W + x] =
            (uint8_t)clampf(top + (bottom - top) * ty, 0.0f, 255.0f);
        }
      }
    }
    snapshot.flowSpeed = bulkFlowSpeed_;
  }

private:
  static constexpr int PARTICLE_COUNT = FLUID_PARTICLE_COUNT;
  static constexpr int PARTICLE_MAX = FLUID_PARTICLE_MAX;
  static constexpr int SOLVER_W = FLUID_SOLVER_W;
  static constexpr int SOLVER_H = FLUID_SOLVER_H;
  static constexpr int SOLVER_CELLS = FLUID_SOLVER_CELLS;
  static constexpr int PAIR_MAX = PARTICLE_COUNT * 24;  // overflow safely falls back to re-walking bins
  static constexpr int NEIGHBOR_REACH = 2;  // h=3.8, bin width=2.0 grid cells
  static constexpr float SPACING = 2.2f;
  static constexpr float SMOOTH_RADIUS = 3.8f;
  static constexpr float SMOOTH_RADIUS_SQ = SMOOTH_RADIUS * SMOOTH_RADIUS;
  static constexpr float TIME_SCALE = 1.0f;
  static constexpr float SIM_DT_MAX = 0.020f;
  static constexpr float GRAVITY_ACCEL = 80.0f;
  static constexpr float K_PRESSURE = 900.0f;
  static constexpr float K_NEAR_PRESSURE = 1800.0f;
  static constexpr float VISC_SIGMA = 10.0f;
  static constexpr float VISC_BETA = 0.10f;
  static constexpr float MAX_DISPLACEMENT = 0.35f;
  static constexpr float MAX_SPEED = 80.0f;
  static constexpr float AIR_DRAG = 0.12f;
  static constexpr float WALL_MARGIN = 0.20f;
  static constexpr float WALL_RESTITUTION = 0.10f;
  static constexpr float WALL_TANGENTIAL_KEEP = 0.96f;
  static constexpr float IMPULSE_TO_VELOCITY = 0.85f;
  static constexpr float DENSITY_EDGE_FRACTION = 0.30f;
  static constexpr float DENSITY_SELF_BIAS = 0.80f;

  struct Particle {
    float x, y;
    float vx, vy;
    float oldX, oldY;
    float density, nearDensity;  // overwritten with pressure after density calculation
    float viscDX, viscDY;
    uint8_t cellX, cellY;
    int16_t nextInCell;
  };

  Particle particles_[PARTICLE_MAX];
  int16_t cellHead_[SOLVER_CELLS];
  uint16_t pairNeighbor_[PAIR_MAX];
  uint16_t pairOffset_[PARTICLE_MAX + 1];
  uint8_t densityField_[SOLVER_CELLS];
  float downX_, downY_;
  float restDensity_;
  float bulkFlowSpeed_;
  uint16_t pairCount_;
  bool pairListValid_;

  static int index(int x, int y) { return y * SOLVER_W + x; }

  static float clampf(float value, float low, float high) {
    return value < low ? low : (value > high ? high : value);
  }

  static void normalize(float &x, float &y) {
    const float magnitude = sqrtf(x * x + y * y);
    if (magnitude > 0.0001f) {
      x /= magnitude;
      y /= magnitude;
    } else {
      x = 0.0f;
      y = 1.0f;
    }
  }

  static void clampVelocity(float &vx, float &vy) {
    const float speedSq = vx * vx + vy * vy;
    if (speedSq > MAX_SPEED * MAX_SPEED) {
      const float scale = MAX_SPEED / sqrtf(speedSq);
      vx *= scale;
      vy *= scale;
    }
  }

  uint32_t randomState_;

  float randomSigned() {
    randomState_ ^= randomState_ << 13;
    randomState_ ^= randomState_ >> 17;
    randomState_ ^= randomState_ << 5;
    return (float)(randomState_ & 0x00FFFFFFu) * (2.0f / 16777215.0f) - 1.0f;
  }

  void clear() {
    memset(particles_, 0, sizeof(particles_));
    memset(cellHead_, 0xFF, sizeof(cellHead_));
    memset(pairNeighbor_, 0, sizeof(pairNeighbor_));
    memset(pairOffset_, 0, sizeof(pairOffset_));
    memset(densityField_, 0, sizeof(densityField_));
    downX_ = 0.0f;
    downY_ = 1.0f;
    restDensity_ = 0.0f;
    bulkFlowSpeed_ = 0.0f;
    pairCount_ = 0;
    pairListValid_ = true;
    randomState_ = 0x2545F491u;
  }

  void calibrateRestDensity() {
    const int reach = (int)ceilf(SMOOTH_RADIUS / SPACING);
    float rho = 0.0f;
    for (int dy = -reach; dy <= reach; ++dy) {
      for (int dx = -reach; dx <= reach; ++dx) {
        if (dx == 0 && dy == 0) continue;
        const float rx = SPACING * (float)dx;
        const float ry = SPACING * (float)dy;
        const float r2 = rx * rx + ry * ry;
        if (r2 >= SMOOTH_RADIUS_SQ) continue;
        const float q = 1.0f - sqrtf(r2) / SMOOTH_RADIUS;
        rho += q * q;
      }
    }
    restDensity_ = rho;
  }

  void seedPool() {
    // 18 by 7 at the rest spacing gives 126 particles, roughly 38% of the
    // 40x40 display area. Small deterministic jitter prevents a frozen lattice.
    const int columns = 25;
    const float x0 = 0.5f * ((float)FLUID_GRID_W - (float)(columns - 1) * SPACING);
    const float y0 = (float)FLUID_GRID_H - x0;
    for (int i = 0; i < PARTICLE_COUNT; ++i) {
      const int col = i % columns;
      const int row = i / columns;
      Particle &p = particles_[i];
      p.x = x0 + (float)col * SPACING + randomSigned() * 0.035f;
      p.y = y0 - (float)row * SPACING + randomSigned() * 0.035f;
      p.vx = 0.0f;
      p.vy = 0.0f;
      p.oldX = p.x;
      p.oldY = p.y;
    }
  }

  void rebuildGrid() {
    memset(cellHead_, 0xFF, sizeof(cellHead_));
    for (int i = 0; i < PARTICLE_COUNT; ++i) {
      Particle &p = particles_[i];
      int cx = (int)(p.x / (float)FLUID_SOLVER_SCALE);
      int cy = (int)(p.y / (float)FLUID_SOLVER_SCALE);
      if (cx < 0) cx = 0;
      if (cx >= SOLVER_W) cx = SOLVER_W - 1;
      if (cy < 0) cy = 0;
      if (cy >= SOLVER_H) cy = SOLVER_H - 1;
      p.cellX = (uint8_t)cx;
      p.cellY = (uint8_t)cy;
      const int c = index(cx, cy);
      p.nextInCell = cellHead_[c];
      cellHead_[c] = (int16_t)i;
    }
  }

  template <typename PairFunction>
  void visitNeighbors(int i, PairFunction visit) {
    const Particle &p = particles_[i];
    const int x0 = p.cellX > NEIGHBOR_REACH ? p.cellX - NEIGHBOR_REACH : 0;
    const int y0 = p.cellY > NEIGHBOR_REACH ? p.cellY - NEIGHBOR_REACH : 0;
    const int x1 = p.cellX + NEIGHBOR_REACH < SOLVER_W ?
                   p.cellX + NEIGHBOR_REACH : SOLVER_W - 1;
    const int y1 = p.cellY + NEIGHBOR_REACH < SOLVER_H ?
                   p.cellY + NEIGHBOR_REACH : SOLVER_H - 1;
    for (int cy = y0; cy <= y1; ++cy) {
      for (int cx = x0; cx <= x1; ++cx) {
        int j = cellHead_[index(cx, cy)];
        while (j >= 0) {
          if (j > i) visit(j);
          j = particles_[j].nextInCell;
        }
      }
    }
  }

  void computeDensityAndViscosity(float dt) {
    for (int i = 0; i < PARTICLE_COUNT; ++i) {
      Particle &p = particles_[i];
      p.density = 0.0f;
      p.nearDensity = 0.0f;
      p.viscDX = 0.0f;
      p.viscDY = 0.0f;
    }
    pairCount_ = 0;
    pairListValid_ = true;

    for (int i = 0; i < PARTICLE_COUNT; ++i) {
      Particle &a = particles_[i];
      pairOffset_[i] = pairCount_;
      visitNeighbors(i, [&](int j) {
        Particle &b = particles_[j];
        const float dx = b.x - a.x;
        const float dy = b.y - a.y;
        const float r2 = dx * dx + dy * dy;
        if (r2 >= SMOOTH_RADIUS_SQ || r2 < 1.0e-8f) return;

        if (pairCount_ < PAIR_MAX) {
          pairNeighbor_[pairCount_++] = (uint16_t)j;
        } else {
          pairListValid_ = false;
        }

        const float r = sqrtf(r2);
        const float invR = 1.0f / r;
        const float ux = dx * invR;
        const float uy = dy * invR;
        const float q = 1.0f - r / SMOOTH_RADIUS;
        const float q2 = q * q;
        const float q3 = q2 * q;
        a.density += q2;
        b.density += q2;
        a.nearDensity += q3;
        b.nearDensity += q3;

        // Clavet viscosity damps only approaching pairs; the equal and opposite
        // position impulses conserve the pair's momentum.
        if (dt > 0.0f) {
          const float closing = (a.vx - b.vx) * ux + (a.vy - b.vy) * uy;
          if (closing > 0.0f) {
            float dv = 0.5f * dt * q *
                       (VISC_SIGMA * closing + VISC_BETA * closing * closing);
            if (dv > 0.5f * closing) dv = 0.5f * closing;
            const float impulse = dv * dt;
            a.viscDX -= impulse * ux;
            a.viscDY -= impulse * uy;
            b.viscDX += impulse * ux;
            b.viscDY += impulse * uy;
          }
        }
      });
      pairOffset_[i + 1] = pairCount_;
    }

    if (!pairListValid_) {
      // A crowded exceptional state can exceed the cache; the density pass is
      // still complete and relaxation will simply re-walk its spatial bins.
      pairCount_ = 0;
    }
    for (int i = 0; i < PARTICLE_COUNT; ++i) {
      particles_[i].x += particles_[i].viscDX;
      particles_[i].y += particles_[i].viscDY;
    }
  }

  void relaxPair(int i, int j, float xi, float yi, float &moveX, float &moveY,
                 float dt2) {
    Particle &a = particles_[i];
    Particle &b = particles_[j];
    const float dx = b.x - xi;
    const float dy = b.y - yi;
    const float r2 = dx * dx + dy * dy;
    if (r2 >= SMOOTH_RADIUS_SQ || r2 < 1.0e-8f) return;
    const float r = sqrtf(r2);
    const float invR = 1.0f / r;
    const float ux = dx * invR;
    const float uy = dy * invR;
    const float q = 1.0f - r / SMOOTH_RADIUS;
    const float q2 = q * q;
    float d = 0.5f * dt2 *
              ((a.density + b.density) * q +
               (a.nearDensity + b.nearDensity) * q2);
    d = clampf(d, -MAX_DISPLACEMENT, MAX_DISPLACEMENT);
    const float sx = ux * d;
    const float sy = uy * d;
    b.x += sx;
    b.y += sy;
    moveX -= sx;
    moveY -= sy;
  }

  void relaxPositions(float dt2) {
    for (int i = 0; i < PARTICLE_COUNT; ++i) {
      Particle &p = particles_[i];
      // Reuse the two density slots for pressure, reducing per-particle RAM.
      p.density = K_PRESSURE * (p.density - restDensity_);
      p.nearDensity = K_NEAR_PRESSURE * p.nearDensity;
    }

    for (int i = 0; i < PARTICLE_COUNT; ++i) {
      Particle &p = particles_[i];
      const float xi = p.x;
      const float yi = p.y;
      float moveX = 0.0f;
      float moveY = 0.0f;
      if (pairListValid_) {
        for (uint16_t k = pairOffset_[i]; k < pairOffset_[i + 1]; ++k) {
          relaxPair(i, pairNeighbor_[k], xi, yi, moveX, moveY, dt2);
        }
      } else {
        visitNeighbors(i, [&](int j) {
          relaxPair(i, j, xi, yi, moveX, moveY, dt2);
        });
      }
      p.x += moveX;
      p.y += moveY;
    }
  }

  void resolveWalls(Particle &p) {
    const float maxX = (float)FLUID_GRID_W - WALL_MARGIN;
    const float maxY = (float)FLUID_GRID_H - WALL_MARGIN;
    if (p.x < WALL_MARGIN) {
      p.x = WALL_MARGIN;
      if (p.vx < 0.0f) p.vx = -p.vx * WALL_RESTITUTION;
      p.vy *= WALL_TANGENTIAL_KEEP;
    } else if (p.x > maxX) {
      p.x = maxX;
      if (p.vx > 0.0f) p.vx = -p.vx * WALL_RESTITUTION;
      p.vy *= WALL_TANGENTIAL_KEEP;
    }
    if (p.y < WALL_MARGIN) {
      p.y = WALL_MARGIN;
      if (p.vy < 0.0f) p.vy = -p.vy * WALL_RESTITUTION;
      p.vx *= WALL_TANGENTIAL_KEEP;
    } else if (p.y > maxY) {
      p.y = maxY;
      if (p.vy > 0.0f) p.vy = -p.vy * WALL_RESTITUTION;
      p.vx *= WALL_TANGENTIAL_KEEP;
    }
  }

  float sampleDensityAt(float x, float y) const {
    int cx = (int)(x / (float)FLUID_SOLVER_SCALE);
    int cy = (int)(y / (float)FLUID_SOLVER_SCALE);
    if (cx < 0) cx = 0;
    if (cx >= SOLVER_W) cx = SOLVER_W - 1;
    if (cy < 0) cy = 0;
    if (cy >= SOLVER_H) cy = SOLVER_H - 1;
    const int x0 = cx > NEIGHBOR_REACH ? cx - NEIGHBOR_REACH : 0;
    const int y0 = cy > NEIGHBOR_REACH ? cy - NEIGHBOR_REACH : 0;
    const int x1 = cx + NEIGHBOR_REACH < SOLVER_W ?
                   cx + NEIGHBOR_REACH : SOLVER_W - 1;
    const int y1 = cy + NEIGHBOR_REACH < SOLVER_H ?
                   cy + NEIGHBOR_REACH : SOLVER_H - 1;
    float rho = 0.0f;
    for (int by = y0; by <= y1; ++by) {
      for (int bx = x0; bx <= x1; ++bx) {
        int j = cellHead_[index(bx, by)];
        while (j >= 0) {
          const float dx = particles_[j].x - x;
          const float dy = particles_[j].y - y;
          const float r2 = dx * dx + dy * dy;
          if (r2 < SMOOTH_RADIUS_SQ) {
            const float q = 1.0f - sqrtf(r2) / SMOOTH_RADIUS;
            rho += q * q;
          }
          j = particles_[j].nextInCell;
        }
      }
    }
    return rho;
  }

  void updateBulkMotion() {
    float speedSquared = 0.0f;
    for (int i = 0; i < PARTICLE_COUNT; ++i) {
      const Particle &p = particles_[i];
      speedSquared += p.vx * p.vx + p.vy * p.vy;
    }
    bulkFlowSpeed_ = sqrtf(speedSquared / (float)PARTICLE_COUNT);
  }
};
