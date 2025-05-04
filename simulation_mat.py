import numpy as np
import pygame
import cupy as cp

# Hardcoded constants
PRESSURE_CONSTANT = 1e10
DRAG = 0.998  # Drag coefficient for velocity damping

class ParticleSystem:
    def __init__(self, count, radius, spacing, width, height):
        self.N = count
        self.radius = radius
        self.bounds = (width, height)

        # GPU arrays
        self.positions = cp.zeros((count, 2), dtype=cp.float32)
        self.velocities = cp.zeros((count, 2), dtype=cp.float32)
        self.forces = cp.zeros((count, 2), dtype=cp.float32)
        self.densities = cp.zeros(count, dtype=cp.float32)
        self.pressures = cp.zeros(count, dtype=cp.float32)

        # SPH constants
        self._calculate_sph_constants(count, width, height)
        self.smoothing_length = radius * 10 # OVERWRITE VALUE IN _calculate_sph_constants
        # Initialize positions
        self._init_positions_grid(spacing)

    def _calculate_sph_constants(self, count, width, height):
        area_per_particle = (width * height * 0.5) / count
        particle_spacing = area_per_particle ** 0.5

        self.smoothing_length = particle_spacing  # tune as needed
        """DEBUGGING
        print(f"Particle spacing: {particle_spacing}, Smoothing length: {self.smoothing_length}")
        """

        self.cubic_volume = cp.pi * self.smoothing_length ** 8 / 4.0
        w0 = (self.smoothing_length ** 2) ** 3 / self.cubic_volume
        self.rest_density = 20.0 * w0  # tune as needed

    def _init_positions_grid(self, spacing):
        grid_w = int(cp.sqrt(self.N))
        start_x = self.bounds[0] // 2 - (grid_w // 2) * spacing
        start_y = self.bounds[1] // 2 - (grid_w // 2) * spacing

        for i in range(self.N):
            row = i // grid_w
            col = i % grid_w
            x = start_x + col * spacing
            y = start_y + row * spacing
            if x < self.bounds[0] and y < self.bounds[1]:
                self.positions[i] = cp.array([x, y])

    def xsph_correction(self):
        """Apply XSPH velocity smoothing to prevent endless particle bouncing."""
        eps = 0.05  # smoothing strength (small)
        diffs = self.positions[:, None, :] - self.positions[None, :, :]
        dists = cp.linalg.norm(diffs, axis=2) + 1e-8

        vel_diffs = self.velocities[:, None, :] - self.velocities[None, :, :]
        lap_w = self.kernel_laplacian(dists)

        densities_sum = self.densities[:, None] + self.densities[None, :] + 1e-8

        correction = eps * cp.sum(vel_diffs * lap_w[:, :, None] / densities_sum[:, :, None], axis=1)
        self.velocities -= correction * 2

    def update(self, dt):
        self.update_density()
        self.update_pressure()
        self.update_forces()
        #self.clamp_velocities()
        self.xsph_correction()
        self.integrate(dt)
        """DEBUGGING
        print(self.densities.get())
        """

    def apply_random_kick(self, magnitude=0.5):
        """Apply random initial velocities to all particles."""
        random_velocities = cp.random.uniform(-magnitude, magnitude, size=(self.N, 2)).astype(cp.float32)
        self.velocities += random_velocities

    def draw(self, screen):
        positions_np = self.positions.get()
        velocities_np = self.velocities.get()
        speeds = np.linalg.norm(velocities_np, axis=1)

        max_speed = 500.0

        for pos, speed in zip(positions_np, speeds):
            # Normalize speed
            t = min(speed / max_speed, 1.0)

            # Map speed to color: blue (cold), red (hot)
            r = int(255 * t)
            g = int(255 * (1 - t))
            b = int(255 * (1 - t))

            pygame.draw.circle(screen, (r, g, b), pos.astype(int), int(self.radius))


    # --- GPU SPH Operations ---

    def kernel(self, r):
        """Poly6 Kernel"""
        result = cp.maximum(0.0, (self.smoothing_length**2 - r**2))**3
        normalization = 315.0 / (64.0 * cp.pi * self.smoothing_length**9)
        return normalization * result

    def kernel_gradient(self, r_vec, r_norm):
        factor = -45.0 / (cp.pi * self.smoothing_length**6)
        scalar = (self.smoothing_length - r_norm)**2
        scalar = scalar[:, None]  # ???
        grad = factor * scalar * (r_vec / (r_norm[:, None] + 1e-8))
        grad = cp.where(r_norm[:, None] < self.smoothing_length, grad, 0)
        return grad

    def kernel_laplacian(self, r_norm):
        factor = 45.0 / (cp.pi * self.smoothing_length**6)
        lap = factor * (self.smoothing_length - r_norm)
        lap = cp.where(r_norm < self.smoothing_length, lap, 0)
        return lap

    def update_density(self):
        diffs = self.positions[:, None, :] - self.positions[None, :, :]
        dists = cp.linalg.norm(diffs, axis=2)

        mask = dists < self.smoothing_length
        density_contributions = self.kernel(dists) * mask

        self.densities = cp.sum(density_contributions, axis=1)
        """DEBUGGING
        if cp.any(self.densities < 1e-2):
            print(f"[WARNING] Extremely low density detected! Min density: {cp.min(self.densities).get()}")
        """
        self.densities = cp.maximum(self.densities, 1.0)

        


    def update_pressure(self):
        self.pressures = cp.maximum(0, PRESSURE_CONSTANT * (self.densities - self.rest_density))

    def update_forces(self):
        diffs = self.positions[:, None, :] - self.positions[None, :, :]
        dists = cp.linalg.norm(diffs, axis=2) + 1e-8

        # Pressure force
        grad_w = self.kernel_gradient(diffs.reshape(-1, 2), dists.flatten()).reshape(self.N, self.N, 2)
        pressures_sum = self.pressures[:, None] + self.pressures[None, :]
        densities_product = (self.densities[:, None] * self.densities[None, :]) + 1e-8

        pressure_terms = -0.5 * pressures_sum / densities_product
        force_pressure = cp.sum(grad_w * pressure_terms[:, :, None], axis=1)

        # Viscosity force
        vel_diffs = self.velocities[:, None, :] - self.velocities[None, :, :]
        lap_w = self.kernel_laplacian(dists)
        viscosity_coefficient = 5.0
        force_viscosity = viscosity_coefficient * cp.sum(vel_diffs * lap_w[:, :, None] / densities_product[:, :, None], axis=1)

        # Gravity force
        gravity_strength = 0.0
        gravity = cp.array([0, gravity_strength], dtype=cp.float32)
        gravity_forces = self.densities[:, None] * gravity

        # Final sum
        self.forces = force_pressure + force_viscosity + gravity_forces


    def clamp_velocities(self, max_speed=500.0):
        speeds = cp.linalg.norm(self.velocities, axis=1)
        mask = speeds > max_speed
        self.velocities[mask] *= (max_speed / (speeds[mask] + 1e-8))[:, None]

    def integrate(self, dt):
        self.velocities += (self.forces / (self.densities[:, None] + 1e-8)) * dt
        self.velocities *= DRAG
        self.positions += self.velocities * dt

        # Boundary conditions
        w, h = self.bounds
        self.positions[:, 0] = cp.clip(self.positions[:, 0], self.radius, w - self.radius)
        self.positions[:, 1] = cp.clip(self.positions[:, 1], self.radius, h - self.radius)

        bounce_x = (self.positions[:, 0] == self.radius) | (self.positions[:, 0] == w - self.radius)
        bounce_y = (self.positions[:, 1] == self.radius) | (self.positions[:, 1] == h - self.radius)

        self.velocities[bounce_x, 0] *= -1
        self.velocities[bounce_y, 1] *= -1
