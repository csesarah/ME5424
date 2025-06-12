import numpy as np

# Boid parameters
BOID_SPEED = 3.0

COHESION_WEIGHT = 0.01
SEPARATION_WEIGHT = 0.02
ALIGNMENT_WEIGHT = 0.03

COMM_DISTANCE = 100  # Distance beyond which clusters form
FEAR_DISTANCE = 90   # Distance at which to avoid drones
PANIC_DISTANCE = 10  # Distance at which to avoid drones with random behavior
BOID_DRONE_REPULSION_WEIGHT = 2.2

class Boid:
    def __init__(self, x, y):
        self.position = np.array([x, y], dtype=np.float64)
        angle = np.random.uniform(0, 2 * np.pi)
        self.velocity = np.array([np.cos(angle), np.sin(angle)]) * BOID_SPEED

    def update(self, boids, drones):
        acceleration = np.zeros(2)
        
        # Flocking rules: cohesion, separation, alignment.
        neighbors = [b for b in boids if np.linalg.norm(self.position - b.position) < COMM_DISTANCE]
        if neighbors:
            # Cohesion: steer toward the average position of neighbors.
            center = np.mean([b.position for b in neighbors], axis=0)
            acceleration += (center - self.position) * COHESION_WEIGHT
        
        # Separation: avoid crowding neighbors.
        for b in neighbors:
            distance = np.linalg.norm(self.position - b.position)
            if distance < 20:
                acceleration -= (b.position - self.position) * SEPARATION_WEIGHT
        
        # Alignment: match velocity to neighbors.
        if neighbors:
            avg_velocity = np.mean([b.velocity for b in neighbors], axis=0)
            acceleration += (avg_velocity - self.velocity) * ALIGNMENT_WEIGHT
        
        # React to drones, stronger repulsion from drones
        for drone in drones:
            d_dist = np.linalg.norm(self.position - drone.position)
            if d_dist < PANIC_DISTANCE:
                # Erratic behavior when too close to a drone
                random_angle = np.random.uniform(0, 2 * np.pi)
                acceleration += np.array([np.cos(random_angle), np.sin(random_angle)]) * BOID_DRONE_REPULSION_WEIGHT/2
            elif d_dist < FEAR_DISTANCE:
                # Strong repulsion 
                acceleration -= (drone.position - self.position) * BOID_DRONE_REPULSION_WEIGHT
        
        self.velocity = limit_speed(self.velocity + acceleration, BOID_SPEED)
        self.position += self.velocity
        self.position = np.clip(self.position, [0,0], [WIDTH, HEIGHT])