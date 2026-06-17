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
        
        # Action space: Continuous float between -1.0 and 1.0 for (X, Y, Z)
        # This is the standard, optimized format for PPO / neural networks
        self.action_space = spaces.Box(
            low=-1.0, 
            high=1.0, 
            shape=(3,), 
            dtype=np.float32
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
        try:
            data = self.sock.recv(4096).decode('utf-8')
            if not data:
                raise ConnectionError("Received empty byte packet from Godot.")
                
            response = json.loads(data)
            
            # Print warnings sent by the Godot Bridge
            if "error" in response and response["error"] != "":
                print(f"\n[Godot Warning]: {response['error']}")
                print("Make sure your drone is spawned and has been added to the 'drone' group!\n")

            obs = np.array([
                response["position"][0], response["position"][1], response["position"][2],
                response["velocity"][0], response["velocity"][1], response["velocity"][2],
                response["coverage"]
            ], dtype=np.float32)
            
            return obs, response["reward"], response["done"]
            
        except (ConnectionError, socket.error) as e:
            print("\n" + "="*50)
            print("ERROR: Connection with the Godot instance was lost.")
            print("This usually means Godot crashed or was stopped.")
            print("Please check the Godot Editor debugger/console window for errors.")
            print("="*50 + "\n")
            raise e
        except json.JSONDecodeError as e:
            print("\n" + "="*50)
            print("ERROR: Received invalid data from Godot.")
            print(f"Raw data received: '{data}'")
            print("="*50 + "\n")
            raise e

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        
        # Send reset command
        msg = json.dumps({"type": "reset"})
        self.sock.sendall(msg.encode('utf-8'))
        
        obs, _, _ = self._get_response()
        info = {}
        return obs, info

    def step(self, action):
        # Scale the action from [-1.0, 1.0] to [0, grid_size - 1]
        scaled_action = (action + 1.0) / 2.0 * (self.grid_size - 1)
        act = np.clip(scaled_action, 0, self.grid_size - 1).astype(int).tolist()
        
        print(f"Current Waypoint Action: {act}")
        
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
    print("Connecting to Godot instance...")
    env = GodotDroneEnv()
    
    # Define the PPO Agent
    model = PPO("MlpPolicy", env, verbose=1, learning_rate=3e-4)
    
    # Start Training
    print("Starting Training...")
    model.learn(total_timesteps=100000)
    
    # Save the trained weight file
    model.save("drone_mapping_agent")
    print("Model Saved.")