import os
from godot_rl.core.godot_env import GodotEnv
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv # Added for compatibility
from godot_rl.wrappers.stable_baselines_wrapper import StableBaselinesGodotEnv# ==========================================
# CONFIGURATION
# ==========================================
# Set env_path=None to connect directly to the active running Godot Editor (recommended for testing!)
# Set env_path="/Applications/Godot.app/Contents/MacOS/Godot" to launch a standalone build.
GODOT_EXE_PATH = None 

# Speed up Godot's internal physics engine (e.g., 8x speed) for much faster training
PHYSICS_SPEEDUP = 8 
# ==========================================

if __name__ == "__main__":
    env = StableBaselinesGodotEnv(env_path="/Applications/Godot.app/Contents/MacOS/Godot", speedup=8)

# 2. Convert it into a valid Gymnasium environment
    # env = GodotEnvGymnasiumWrapper(raw_env)
    # Wrap the environment in a DummyVecEnv to resolve legacy Gym/Gymnasium type conflicts
    # env = DummyVecEnv([lambda: GodotEnv(env_path=GODOT_EXE_PATH, speedup=PHYSICS_SPEEDUP)])
    
    try:
        # Define the PPO Agent
        print("Initializing PPO Agent...")
        model = PPO("MlpPolicy", env, verbose=1, learning_rate=3e-4)
        
        # Start Training
        print("Starting Training with Godot RL Agents...")
        model.learn(total_timesteps=100000)
        
        # Save Model
        model.save("godot_rl_drone_agent")
        print("Model Saved.")
        
    finally:
        env.close()