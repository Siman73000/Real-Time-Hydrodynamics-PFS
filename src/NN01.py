import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import TensorDataset, DataLoader
import json
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import os
import glob
from PIL import Image
import numba  # Numba for JIT compilation

# -------------------------------
# Configuration for Particle Count
# -------------------------------
GENERATE_RANDOM_PARTICLES = True
NUM_PARTICLES = 10**4

# -------------------------------
# Set device for GPU usage
# -------------------------------
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print("Using device:", device)

# -------------------------------
# Define NN01: ParticleInformationNet (Processes 3D particle positions)
# -------------------------------
class ParticleInformationNet(nn.Module):
    def __init__(self):
        super(ParticleInformationNet, self).__init__()
        self.net = nn.Sequential(
            nn.Linear(3, 128),
            nn.ReLU(),
            nn.Linear(128, 64),
            nn.ReLU(),
            # Output 6 features (first 3 used as NN-based velocity update)
            nn.Linear(64, 6)
        )
        
    def forward(self, x):
        return self.net(x)

# -------------------------------
# Define NN02: NavierStokesApproximator (Refines particle motion)
# -------------------------------
class NavierStokesApproximator(nn.Module):
    def __init__(self, input_dim=6, output_dim=6):
        super(NavierStokesApproximator, self).__init__()
        self.net = nn.Sequential(
            nn.Linear(input_dim, 128),
            nn.ReLU(),
            nn.Linear(128, 64),
            nn.ReLU(),
            nn.Linear(64, output_dim)
        )
        
    def forward(self, x):
        return self.net(x)

# -------------------------------
# Define dummy networks for water appearance (used only for training)
# -------------------------------
class WaterAppearanceNet(nn.Module):
    def __init__(self):
        super(WaterAppearanceNet, self).__init__()
        self.features = nn.Sequential(
            nn.Conv2d(3, 32, kernel_size=3, padding=1),
            nn.ReLU(),
            nn.MaxPool2d(2),
            nn.Conv2d(32, 64, kernel_size=3, padding=1),
            nn.ReLU(),
            nn.MaxPool2d(2),
            nn.Conv2d(64, 128, kernel_size=3, padding=1),
            nn.ReLU(),
            nn.MaxPool2d(2)
        )
        self.classifier = nn.Sequential(
            nn.Linear(128 * 32 * 32, 256),
            nn.ReLU(),
            nn.Linear(256, 128)
        )
        
    def forward(self, x):
        x = self.features(x)
        x = x.view(x.size(0), -1)
        return self.classifier(x)

class WaterVisualizationNet(nn.Module):
    def __init__(self, sim_feature_dim=6, appearance_feature_dim=128):
        super(WaterVisualizationNet, self).__init__()
        self.fc = nn.Sequential(
            nn.Linear(sim_feature_dim + appearance_feature_dim, 512 * 16 * 16),
            nn.ReLU(),
        )
        self.deconv = nn.Sequential(
            nn.ConvTranspose2d(512, 256, kernel_size=4, stride=2, padding=1),
            nn.ReLU(),
            nn.ConvTranspose2d(256, 128, kernel_size=4, stride=2, padding=1),
            nn.ReLU(),
            nn.ConvTranspose2d(128, 64, kernel_size=4, stride=2, padding=1),
            nn.ReLU(),
            nn.ConvTranspose2d(64, 3, kernel_size=4, stride=2, padding=1),
            nn.Tanh()  # outputs in [-1, 1]
        )
        
    def forward(self, sim_features, appearance_features):
        x = torch.cat([sim_features, appearance_features], dim=1)
        x = self.fc(x)
        x = x.view(x.size(0), 512, 16, 16)
        x = self.deconv(x)
        return x

# -------------------------------
# Composite Model for Full Training (includes appearance networks)
# -------------------------------
class FullWaterSimulationModel(nn.Module):
    def __init__(self, nn01, nn02, nn03, nn04):
        super(FullWaterSimulationModel, self).__init__()
        self.nn01 = nn01  # Particle information network
        self.nn02 = nn02  # Navier-Stokes approximator
        self.nn03 = nn03  # Water appearance extractor
        self.nn04 = nn04  # Water visualization generator
        
    def forward(self, particle_data, water_image):
        particle_features = self.nn01(particle_data)
        sim_features = self.nn02(particle_features)
        appearance_features = self.nn03(water_image)
        visualization = self.nn04(sim_features, appearance_features)
        return visualization

# -------------------------------
# Instantiate Networks and the Composite Model
# -------------------------------
nn01 = ParticleInformationNet()
nn02 = NavierStokesApproximator()
nn03 = WaterAppearanceNet()       # For training only
nn04 = WaterVisualizationNet()    # For training only

full_model = FullWaterSimulationModel(nn01, nn02, nn03, nn04).to(device)

# -------------------------------
# Data Loading and Preparation
# -------------------------------
if not GENERATE_RANDOM_PARTICLES:
    with open("particles.json", "r") as f:
        particles = json.load(f)
    data_array = np.array(
        [[p["x"], p["y"], p["z"], p["vx"], p["vy"], p["vz"]] for p in particles],
        dtype=np.float32
    )
    particle_positions = data_array[:, :3]
    simulation_targets = data_array[:, 3:] + 0.1  # Dummy targets for training
    particle_tensor = torch.from_numpy(particle_positions)
    sim_target_tensor = torch.from_numpy(simulation_targets)
else:
    # Generate random particles programmatically.
    particle_tensor = (torch.rand(NUM_PARTICLES, 3) * 2 - 1).float()
    sim_target_tensor = torch.zeros_like(particle_tensor) + 0.1

def load_water_images(image_dir, image_size=(256,256)):
    image_files = glob.glob(os.path.join(image_dir, "*.jpg"))
    images = []
    for file in image_files:
        try:
            img = Image.open(file).convert("RGB")
            img = img.resize(image_size)
            img_np = np.array(img).astype(np.float32)
            img_np = img_np / 127.5 - 1.0  # Normalize to [-1, 1]
            img_np = np.transpose(img_np, (2, 0, 1))
            images.append(torch.tensor(img_np))
        except Exception as e:
            print(f"Error loading {file}: {e}")
    if images:
        return torch.stack(images)
    else:
        raise ValueError("No water images found in the specified folder.")

try:
    water_images = load_water_images("./water_images")
except Exception as e:
    print(e)
    water_images = torch.zeros((1, 3, 256, 256))

num_particles = particle_tensor.size(0)
if water_images.size(0) < num_particles:
    repeats = int(np.ceil(num_particles / water_images.size(0)))
    water_images = water_images.repeat(repeats, 1, 1, 1)
water_images = water_images[:num_particles]
visualization_targets = water_images.clone()

combined_dataset = TensorDataset(particle_tensor, water_images, visualization_targets)
dataloader = DataLoader(combined_dataset, batch_size=8, shuffle=True)

# -------------------------------
# Training the Full Model
# -------------------------------
criterion = nn.MSELoss()
optimizer = optim.Adam(full_model.parameters(), lr=0.001)
num_epochs = 10

print("Starting training...")
for epoch in range(num_epochs):
    running_loss = 0.0
    for particles_batch, images_batch, targets_batch in dataloader:
        particles_batch = particles_batch.to(device)
        images_batch = images_batch.to(device)
        targets_batch = targets_batch.to(device)
        
        optimizer.zero_grad()
        outputs = full_model(particles_batch, images_batch)
        loss = criterion(outputs, targets_batch)
        loss.backward()
        optimizer.step()
        running_loss += loss.item() * particles_batch.size(0)
    
    epoch_loss = running_loss / len(combined_dataset)
    print(f"Epoch [{epoch+1}/{num_epochs}], Loss: {epoch_loss:.10f}")
print("Training complete.")

# -------------------------------
# Build a Simulation Model for 3D Particle Updates
# -------------------------------
# Use only the fluid simulation networks (without the appearance networks)
sim_model = torch.nn.Sequential(nn01, nn02).to(device)
sim_model.eval()

initial_positions = particle_tensor.to(device).float().clone()
particle_tensor_sim = initial_positions.clone()  # positions [N, 3]

# Add a small random perturbation to kickstart motion
velocity_tensor = 0.001 * torch.randn_like(particle_tensor_sim)

# -------------------------------
# Optimized Collision Resolution using Numba
# -------------------------------
@numba.njit(parallel=True)
def resolve_collisions_numba(positions, grid_indices, min_distance, K):
    N = positions.shape[0]
    repulsion = np.zeros_like(positions)
    for i in numba.prange(N):
        ix = grid_indices[i, 0]
        iy = grid_indices[i, 1]
        iz = grid_indices[i, 2]
        for j in range(i+1, N):
            jx = grid_indices[j, 0]
            jy = grid_indices[j, 1]
            jz = grid_indices[j, 2]
            # Only consider particles in neighboring cells.
            if abs(ix - jx) <= 1 and abs(iy - jy) <= 1 and abs(iz - jz) <= 1:
                dx = positions[i, 0] - positions[j, 0]
                dy = positions[i, 1] - positions[j, 1]
                dz = positions[i, 2] - positions[j, 2]
                dist = np.sqrt(dx*dx + dy*dy + dz*dz) + 1e-6
                if dist < min_distance:
                    force_mag = K * (min_distance - dist) ** 2
                    fx = force_mag * dx / dist
                    fy = force_mag * dy / dist
                    fz = force_mag * dz / dist
                    repulsion[i, 0] += fx
                    repulsion[i, 1] += fy
                    repulsion[i, 2] += fz
                    repulsion[j, 0] -= fx
                    repulsion[j, 1] -= fy
                    repulsion[j, 2] -= fz
    return repulsion

def resolve_collisions_optimized(positions, min_distance=0.0999, cell_size=0.1):
    # Compute grid indices based on positions in [-1, 1]
    positions_np = positions.cpu().numpy()
    grid_indices = ((positions_np + 1.0) / cell_size).astype(np.int64)
    K = 1200.0  # stiffness constant for repulsion
    repulsion_np = resolve_collisions_numba(positions_np, grid_indices, min_distance, K)
    return torch.from_numpy(repulsion_np).to(positions.device)

# -------------------------------
# Real-Time 3D Visualization with Vispy
# -------------------------------
from vispy import app, scene

canvas = scene.SceneCanvas(keys='interactive', size=(800, 600), show=True, title='3D Water Particle Fluid Simulation')
view = canvas.central_widget.add_view()
view.camera = scene.TurntableCamera(fov=45, azimuth=30, elevation=30, distance=5)
markers = scene.visuals.Markers()
view.add(markers)

def update_markers():
    pts = particle_tensor_sim.cpu().numpy()
    markers.set_data(pts, face_color=(0, 0.5, 1, 1), size=5)

update_markers()

def update_simulation(event):
    global particle_tensor_sim, velocity_tensor, sim_model, prev_yaw, prev_roll
    try:
        dt = 0.016  # time step (seconds)
        with torch.no_grad():
            # Compute simulation updates from NN and forces.
            sim_output = sim_model(particle_tensor_sim)  # shape [N, 6]
            nn_delta_v = sim_output[:, :3] * 0.1  # scale NN output

            # Log diagnostic information for NN output.
            avg_nn_force = nn_delta_v.norm(dim=1).mean().item()

            # Use the optimized (Numba) collision resolution.
            collision_force = resolve_collisions_optimized(particle_tensor_sim, min_distance=0.0999, cell_size=0.1)

            # Gravity force: constant downward acceleration.
            gravity_force = torch.tensor([0, -9.81, 0], device=device).unsqueeze(0).repeat(particle_tensor_sim.shape[0], 1)

            total_acceleration = nn_delta_v + collision_force + gravity_force

            # Update velocity (semi-implicit Euler integration) with damping.
            velocity_tensor = (velocity_tensor + total_acceleration * dt) * 0.99

            # Log the average speed of particles.
            avg_speed = velocity_tensor.norm(dim=1).mean().item()
            print(f"Avg nn_delta_v: {avg_nn_force:.4f}, Avg speed: {avg_speed:.4f}")

            # Update positions.
            new_positions = particle_tensor_sim + velocity_tensor * dt

            # ---------------------------------------------------------
            # Incorporate camera motion (yaw and roll) into collisions.
            # ---------------------------------------------------------
            # Get current camera angles as float32.
            new_yaw = np.deg2rad(view.camera.azimuth).astype(np.float32)
            new_roll = np.deg2rad(getattr(view.camera, 'roll', 0.0)).astype(np.float32)

            # If first frame, initialize previous angles.
            if 'prev_yaw' not in globals():
                prev_yaw = new_yaw
                prev_roll = new_roll

            # Build the rotation matrices as float32.
            R_yaw = np.array([
                [np.cos(new_yaw), 0, np.sin(new_yaw)],
                [0, 1, 0],
                [-np.sin(new_yaw), 0, np.cos(new_yaw)]
            ], dtype=np.float32)
            R_roll = np.array([
                [1, 0, 0],
                [0, np.cos(new_roll), -np.sin(new_roll)],
                [0, np.sin(new_roll),  np.cos(new_roll)]
            ], dtype=np.float32)
            R = R_yaw.dot(R_roll)
            R_inv = np.linalg.inv(R).astype(np.float32)

            # Transform positions and velocities into the rotated coordinate system.
            positions_np = new_positions.cpu().numpy()
            velocities_np = velocity_tensor.cpu().numpy()
            rotated_positions = positions_np.dot(R.T)
            rotated_velocities = velocities_np.dot(R.T)

            # --- Compute camera-induced extra velocity ---
            # Estimate angular velocity (omega) based on change in yaw and roll.
            d_yaw = new_yaw - prev_yaw
            d_roll = new_roll - prev_roll
            # Angular velocity vector in the rotated coordinate system.
            # Here we assume omega = [d_roll/dt, d_yaw/dt, 0]
            omega = np.array([d_roll / dt, d_yaw / dt, 0], dtype=np.float32)
            # Compute extra velocity from camera rotation: v = omega x r.
            extra_velocity = np.cross(omega, rotated_positions)
            # Add the extra velocity to the rotated velocities.
            rotated_velocities += extra_velocity

            # Update previous angles for next frame.
            prev_yaw = new_yaw
            prev_roll = new_roll

            # Enforce boundary collisions in the rotated space (invisible box [-1, 1]^3).
            for axis in range(3):
                low = -1.0
                high = 1.0
                mask_low = rotated_positions[:, axis] < low
                mask_high = rotated_positions[:, axis] > high
                rotated_velocities[mask_low, axis] *= -0.5
                rotated_velocities[mask_high, axis] *= -0.5
                rotated_positions[:, axis] = np.clip(rotated_positions[:, axis], low, high)

            # Transform the corrected positions and velocities back to world coordinates.
            new_positions_world = rotated_positions.dot(R_inv.T)
            new_velocities_world = rotated_velocities.dot(R_inv.T)

            particle_tensor_sim = torch.from_numpy(new_positions_world).to(device)
            velocity_tensor = torch.from_numpy(new_velocities_world).to(device)
        update_markers()
        canvas.update()
    except Exception as e:
        print("Error in update_simulation:", e)


timer = app.Timer(interval=0.016, connect=update_simulation, start=True)

@canvas.events.key_press.connect
def on_key_press(event):
    global particle_tensor_sim, velocity_tensor
    if event.key in ("Enter", "Return"):
        print("Restarting simulation with new perturbation.")
        particle_tensor_sim = initial_positions.clone()
        # Add a small random perturbation to help kickstart motion.
        velocity_tensor = 0.001 * torch.randn_like(particle_tensor_sim)

if __name__ == '__main__':
    app.run()
