"""
Results Visualization for INS_Mechanization
Author: Mohammed Hsiny
Year: 2026

Reads C++ output files and generates publication-ready figures.
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os
import sys

def load_results(result_dir):
    """Load trajectory and error files from result directory."""
    files = {
        'estimated': None,
        'truth': None,
        'error': None
    }
    
    # Try common file names
    candidates = {
        'estimated': ['trajectory_estimated.txt', 'EstimatedTrajectory.txt',
                      'Result.txt', 'result.txt', 'estimated.nav'],
        'truth': ['trajectory_truth.txt', 'TruthTrajectory.txt',
                  'Truth.txt', 'true.txt', 'truth.nav'],
        'error': ['error.txt', 'Error.txt', 'Diff.txt', 'diff_result.txt']
    }
    
    # 1. Check for combined denu_result.txt (Time, True_N, True_E, True_U, Our_N, Our_E, Our_U)
    denu_path = os.path.join(result_dir, 'denu_result.txt')
    if os.path.exists(denu_path):
        try:
            denu_data = np.loadtxt(denu_path)
            if denu_data.ndim == 2 and denu_data.shape[1] >= 7:
                # Column mapping: True_E, True_N, True_U and Our_E, Our_N, Our_U
                files['truth'] = np.column_stack([denu_data[:, 2], denu_data[:, 1], denu_data[:, 3]])
                files['estimated'] = np.column_stack([denu_data[:, 5], denu_data[:, 4], denu_data[:, 6]])
                
                # ENU error: Time, dE, dN, dU
                dE = denu_data[:, 5] - denu_data[:, 2]
                dN = denu_data[:, 4] - denu_data[:, 1]
                dU = denu_data[:, 6] - denu_data[:, 3]
                files['error'] = np.column_stack([denu_data[:, 0], dE, dN, dU])
                print(f"[OK] Loaded trajectories and errors from: {denu_path}")
        except Exception as e:
            print(f"[WARN] Could not load denu_result.txt: {e}")

    # 2. Check candidate files if any key is still None
    for key, names in candidates.items():
        if files[key] is not None:
            continue
        for name in names:
            path = os.path.join(result_dir, name)
            if os.path.exists(path):
                try:
                    data = np.loadtxt(path)
                    files[key] = data
                    print(f"[OK] Loaded {key}: {path}")
                    break
                except Exception as e:
                    print(f"[WARN] Could not load {path}: {e}")
    
    return files

def plot_trajectory_3d(files, output_dir):
    """Plot 3D trajectory comparison."""
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    if files['estimated'] is not None:
        data = files['estimated']
        if data.shape[1] >= 3:
            ax.plot(data[:, 0], data[:, 1], data[:, 2],
                    'b-', linewidth=1.5, label='Estimated')
    
    if files['truth'] is not None:
        data = files['truth']
        if data.shape[1] >= 3:
            ax.plot(data[:, 0], data[:, 1], data[:, 2],
                    'r--', linewidth=1.5, label='Ground Truth')
    
    ax.set_xlabel('East (m)')
    ax.set_ylabel('North (m)')
    ax.set_zlabel('Up (m)')
    ax.set_title('3D Trajectory Comparison')
    ax.legend()
    ax.grid(True)
    
    plt.tight_layout()
    path = os.path.join(output_dir, 'trajectory_3d.png')
    plt.savefig(path, dpi=150)
    print(f"[OK] Saved {path}")
    plt.close()

def plot_position_error(files, output_dir):
    """Plot position error over time."""
    if files['error'] is None:
        print("[WARN] No error file found, skipping position error plot")
        return
    
    data = files['error']
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    
    if data.shape[1] >= 4:
        t = data[:, 0]
        axes[0].plot(t, data[:, 1], 'r-', linewidth=1.2)
        axes[0].set_ylabel('East Error (m)')
        axes[0].grid(True)
        
        axes[1].plot(t, data[:, 2], 'g-', linewidth=1.2)
        axes[1].set_ylabel('North Error (m)')
        axes[1].grid(True)
        
        axes[2].plot(t, data[:, 3], 'b-', linewidth=1.2)
        axes[2].set_ylabel('Up Error (m)')
        axes[2].set_xlabel('Time (s)')
        axes[2].grid(True)
    
    fig.suptitle('Position Error Evolution (Strapdown INS Drift)')
    plt.tight_layout()
    path = os.path.join(output_dir, 'position_error.png')
    plt.savefig(path, dpi=150)
    print(f"[OK] Saved {path}")
    plt.close()

def plot_rmse(files, output_dir):
    """Plot RMSE evolution over time."""
    if files['error'] is None:
        print("[WARN] No error file found, skipping RMSE plot")
        return
    
    data = files['error']
    fig, ax = plt.subplots(figsize=(10, 6))
    
    if data.shape[1] >= 4:
        t = data[:, 0]
        pos_error = np.sqrt(data[:, 1]**2 + data[:, 2]**2 + data[:, 3]**2)
        ax.plot(t, pos_error, 'k-', linewidth=1.5, label='3D Position Error')
        
        # Compute moving RMSE
        window = min(100, max(2, len(t) // 10))
        if window > 1:
            rmse = np.array([
                np.sqrt(np.mean(pos_error[max(0, i-window):i+1]**2))
                for i in range(len(t))
            ])
            ax.plot(t, rmse, 'r--', linewidth=1.5, label=f'Moving RMSE (w={window})')
    
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Error (m)')
    ax.set_title('Position Error and RMSE Evolution (Quadratic Growth)')
    ax.legend()
    ax.grid(True)
    
    plt.tight_layout()
    path = os.path.join(output_dir, 'rmse_evolution.png')
    plt.savefig(path, dpi=150)
    print(f"[OK] Saved {path}")
    plt.close()

def main():
    print("=" * 60)
    print("INS_Mechanization - Results Visualization")
    print("Author: Mohammed Hsiny")
    print("Year: 2026")
    print("=" * 60)
    
    # Determine result directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    result_dir = os.path.join(script_dir, '..', 'code', 'Result')
    
    if not os.path.exists(result_dir):
        print(f"[ERROR] Result directory not found: {result_dir}")
        print("Please run the C++ program first to generate results.")
        sys.exit(1)
    
    # Create output directory
    output_dir = os.path.join(script_dir, 'Figures')
    os.makedirs(output_dir, exist_ok=True)
    print(f"[INFO] Output directory: {output_dir}")
    
    # Load results
    print("\n[1/4] Loading results...")
    files = load_results(result_dir)
    
    # Generate plots
    print("\n[2/4] Plotting 3D trajectory...")
    plot_trajectory_3d(files, output_dir)
    
    print("\n[3/4] Plotting position error...")
    plot_position_error(files, output_dir)
    
    print("\n[4/4] Plotting RMSE evolution...")
    plot_rmse(files, output_dir)
    
    print("\n" + "=" * 60)
    print("[OK] All figures generated in:", output_dir)
    print("=" * 60)

if __name__ == "__main__":
    main()
