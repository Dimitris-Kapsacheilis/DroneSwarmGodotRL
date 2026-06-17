import socket
import json
import numpy as np
import gymnasium as gym
from gymnasium import spaces
from stable_baselines3 import PPO

class GodotDroneEnv(gym.Env):
    def __init__(self, ip="127.0.0.1", port=11000):
        super(GodotDroneEnv, self).__init__()
        
        # Grid dimensions (Must match your grid_size in Godot)
        self.grid_size = np.array([100, 100, 100], dtype=np.int32)
        
        # Action space: Discrete 3D Grid coordinate (X, Y, Z) to target next
        self.action_space = spaces.Box(
            low=np.array([0, 0, 0]), 
            high=self.grid_size - 1, 
            dtype=np.int32
        )
        
        # Observation space: Position (3D), Velocity (3D), Coverage (1D)
        self.observation_space = spaces.Box(
            low=np.array([-1e4, -1e4, -1e4, -1e3, -1e3, -1e3, 0.0]),
            high=np.array([1e4, 1e4, 1e4, 1e3, 1e3, 1e3, 100.0]),
            dtype=np.float32
        )
        
        # Establish Socket Connection to Godot Server
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((ip, port))

    def _get_response(self):
        data = self.sock.recv(4096).decode('utf-8')
        response = json.loads(data)
        
        obs = np.array([
            response["position"][0], response["position"][1], response["position"][2],
            response["velocity"][0], response["velocity"][1], response["velocity"][2],
            response["coverage"]
        ], dtype=np.float32)
        
        return obs, response["reward"], response["done"]

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        
        # Send reset command
        msg = json.dumps({"type": "reset"})
        self.sock.sendall(msg.encode('utf-8'))
        
        obs, _, _ = self._get_response()
        info = {}
        return obs, info

    def step(self, action):
        # Convert action floats to integers for grid mapping
        act = action.astype(int).tolist()
        
        # Send target action coordinates
        msg = json.dumps({"type": "action", "action": act})
        self.sock.sendall(msg.encode('utf-8'))
        
        # Wait for the drone to arrive and return simulation data
        obs, reward, done = self._get_response()
        
        truncated = False
        return obs, reward, done, truncated, {}

    def close(self):
        self.sock.close()


# ==========================================
# STABLE-BASELINES3 TRAINING EXECUTION
# ==========================================
if __name__ == "__main__":
    # 1. Run your Godot project first in the Editor so the port is open!
    print("Connecting to Godot instance...")
    env = GodotDroneEnv()
    
    # 2. Define the PPO Agent
    model = PPO("MlpPolicy", env, verbose=1, learning_rate=3e-4)
    
    # 3. Start Training
    print("Starting Training...")
    model.learn(total_timesteps=100000)
    
    # 4. Save the trained weight file
    model.save("drone_mapping_agent")
    print("Model Saved.")