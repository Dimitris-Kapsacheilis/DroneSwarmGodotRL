import argparse
import glob
import json
import os
import sys
from pathlib import Path
import pandas as pd
import plotly.express as px
import matplotlib.pyplot as plt
import seaborn as sns


# ==================== PATH DETECTOR ====================
def get_godot_user_dir(project_name: str = "Drone Swarm") -> Path:
    """Resolves the platform-specific Godot app_userdata directory."""
    home = Path.home()
    if sys.platform == "darwin":  # macOS
        return home / "Library" / "Application Support" / "Godot" / "app_userdata" / project_name
    elif sys.platform == "win32":  # Windows
        return Path(os.environ.get("APPDATA", "")) / "Godot" / "app_userdata" / project_name
    else:  # Linux / Unix
        return home / ".local" / "share" / "godot" / "app_userdata" / project_name


def find_latest_session(base_grid_dir: Path) -> Path:
    """Finds the most recently created session directory."""
    sessions = glob.glob(os.path.join(base_grid_dir, "session_*"))
    if not sessions:
        raise FileNotFoundError(f"No session directories found in {base_grid_dir}")
    # Sort by modification time to get the latest directory
    latest_session = max(sessions, key=os.path.getmtime)
    return Path(latest_session)


# ==================== DATA LOADERS ====================
def load_latest_episode_csv(session_dir: Path, episode_num: int = -1) -> tuple[pd.DataFrame, Path]:
    """
    Loads an episode CSV from the session's csv/ folder.
    By default (episode_num = -1), loads the latest one.
    """
    csv_dir = session_dir / "csv"
    csv_files = glob.glob(os.path.join(csv_dir, "*.csv"))
    if not csv_files:
        raise FileNotFoundError(f"No CSV files found in {csv_dir}")
    
    if episode_num == -1:
        # Load the latest file by modification time
        chosen_file = max(csv_files, key=os.path.getmtime)
    else:
        # Try to match the specific episode number
        match_str = f"episode_{episode_num:04d}.csv"
        matched = [f for f in csv_files if f.endswith(match_str)]
        if not matched:
            raise FileNotFoundError(f"Episode {episode_num} CSV not found in {csv_dir}")
        chosen_file = matched[0]

    df = pd.read_csv(chosen_file)
    return df, Path(chosen_file)


def load_cumulative_json(session_dir: Path) -> tuple[pd.DataFrame, Path]:
    """Loads cumulative visit JSON data from the session folder."""
    cumulative_path = session_dir / "cumulative" / "drone_total_visits.json"
    if not cumulative_path.exists():
        raise FileNotFoundError(f"Cumulative JSON not found at {cumulative_path}")
    
    with open(cumulative_path, "r") as f:
        data = json.load(f)
    
    df = pd.DataFrame(data.get("visits", []))
    return df, cumulative_path


# ==================== PLOTTING FUNCTIONS ====================
def plot_plotly_3d_episode(df: pd.DataFrame, title: str):
    """Plots interactive 3D flight path of a single episode."""
    fig = px.scatter_3d(
        df, x='x', y='y', z='z',
        color=df.index,  # Color indicates step/visit order
        opacity=0.8,
        title=title,
        color_continuous_scale="Viridis"
    )
    fig.update_traces(marker=dict(size=4))
    fig.show()


def plot_plotly_3d_cumulative(df: pd.DataFrame, title: str):
    """Plots interactive 3D heatmap of overall visit frequencies."""
    if df.empty:
        print("Warning: Cumulative visit data is empty.")
        return
        
    fig = px.scatter_3d(
        df, x='x', y='y', z='z',
        color='count',
        size='count',
        size_max=15,
        opacity=0.7,
        title=title,
        color_continuous_scale="Hot"
    )
    fig.show()


def plot_matplotlib_analysis(df: pd.DataFrame, title_suffix: str):
    """Generates detailed 3D trajectory plot and 2D projections."""
    # 3D Path plot
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    scatter = ax.scatter(
        df['x'], df['y'], df['z'],
        c=df.index,
        cmap='viridis',
        s=20, 
        alpha=0.8
    )
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.colorbar(scatter, label='Visit Order (Step)')
    plt.title(f'3D Drone Path - {title_suffix}')
    plt.show()

    # 2D Projection heatmaps
    fig, axes = plt.subplots(2, 2, figsize=(14, 11))

    # XY top-down view
    xy = df.groupby(['x', 'y']).size().reset_index(name='visits')
    xy_pivot = xy.pivot(index='y', columns='x', values='visits').fillna(0)
    sns.heatmap(xy_pivot, ax=axes[0,0], cmap='YlOrRd')
    axes[0,0].set_title('XY Projection (Top View)')

    # XZ side view
    xz = df.groupby(['x', 'z']).size().reset_index(name='visits')
    xz_pivot = xz.pivot(index='z', columns='x', values='visits').fillna(0)
    sns.heatmap(xz_pivot, ax=axes[0,1], cmap='YlOrRd')
    axes[0,1].set_title('XZ Projection')

    # YZ side view
    yz = df.groupby(['y', 'z']).size().reset_index(name='visits')
    yz_pivot = yz.pivot(index='z', columns='y', values='visits').fillna(0)
    sns.heatmap(yz_pivot, ax=axes[1,0], cmap='YlOrRd')
    axes[1,0].set_title('YZ Projection')

    # Cumulative Unique Nodes Covered over Time
    axes[1,1].plot(df.index + 1, range(1, len(df) + 1), 'b-', linewidth=2)
    axes[1,1].set_title('Cumulative Cells Explored over Time')
    axes[1,1].set_xlabel('Simulation Step')
    axes[1,1].set_ylabel('Unique Cells Visited')
    axes[1,1].grid(True, linestyle="--", alpha=0.6)

    plt.suptitle(f"Grid Space Coverage Metrics - {title_suffix}", fontsize=14, fontweight="bold")
    plt.tight_layout()
    plt.show()


# ==================== MAIN LIFECYCLE ====================
def main():
    parser = argparse.ArgumentParser(description="Analyze and visualize 3D Grid Drone coverage data.")
    parser.add_argument("--session", type=str, default="", help="Specific session directory name (optional).")
    parser.add_argument("--episode", type=int, default=-1, help="Episode file number to display. Defaults to latest.")
    parser.add_argument("--cumulative", action="store_true", help="Display cumulative heatmap instead of single episode path.")
    parser.add_argument("--static", action="store_true", help="Use Matplotlib for 2D projections & analytical plots instead of interactive Plotly.")
    
    args = parser.parse_args()

    # Detect user directories
    user_dir = get_godot_user_dir()
    grid_base_dir = user_dir / "Grid_HeatMap"
    
    if not grid_base_dir.exists():
        print(f"Error: Directory not found at: {grid_base_dir}\nMake sure you have run the simulation first.")
        sys.exit(1)

    # Choose session path
    try:
        if args.session:
            session_path = grid_base_dir / args.session
        else:
            session_path = find_latest_session(grid_base_dir)
            print(f"📁 Auto-detected latest session: {session_path.name}")
    except FileNotFoundError as e:
        print(f"Error: {e}")
        sys.exit(1)

    # Load and Plot Data
    if args.cumulative:
        try:
            df, file_p = load_cumulative_json(session_path)
            print(f"📊 Loaded cumulative data from: {file_p.name}")
            plot_plotly_3d_cumulative(df, f"Visit Frequencies - Session: {session_path.name}")
        except FileNotFoundError as e:
            print(f"Error loading cumulative: {e}\n(Cumulative files are only written periodically based on 'save_every_n_episodes')")
    else:
        try:
            df, file_p = load_latest_episode_csv(session_path, args.episode)
            print(f"📈 Loaded {len(df)} entries from: {file_p.parent.name}/{file_p.name}")
            
            title = f"Drone Track Path: {file_p.name} ({session_path.name})"
            if args.static:
                plot_matplotlib_analysis(df, file_p.name)
            else:
                plot_plotly_3d_episode(df, title)
        except FileNotFoundError as e:
            print(f"Error loading episode: {e}")


if __name__ == "__main__":
    main()