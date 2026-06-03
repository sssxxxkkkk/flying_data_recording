import sys

try:
    import numpy as np
    _HAS_NUMPY = True
except ImportError:
    _HAS_NUMPY = False

try:
    import matplotlib.pyplot as plt
    _HAS_MATPLOTLIB = True
except ImportError:
    _HAS_MATPLOTLIB = False

def load_data(filepath):
    """Read data file, return cam, sys, delta as lists of ints"""
    cam = []
    sys_ts = []
    delta = []
    with open(filepath, 'r') as f:
        header = f.readline()  # skip header line
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = line.split()
            if len(parts) < 3:
                continue
            c = int(parts[0])
            s = int(parts[1])
            d = int(parts[2])
            cam.append(c)
            sys_ts.append(s)
            delta.append(d)
    return cam, sys_ts, delta

def stats(arr):
    """Return a dict: sample count, mean, std, min, max, coefficient of variation (%)"""
    if _HAS_NUMPY:
        a = np.array(arr, dtype=np.float64)
        n = len(a)
        if n < 2:
            return {'n': n, 'mean': float(a[0]) if n == 1 else 0, 'std': 0,
                    'min': float(a[0]) if n == 1 else 0, 'max': float(a[0]) if n == 1 else 0, 'cv%': 0}
        mean = np.mean(a)
        std = np.std(a, ddof=1)  # sample standard deviation
        minv = np.min(a)
        maxv = np.max(a)
        cv = (std / mean * 100) if mean != 0 else 0.0
        return {'n': n, 'mean': mean, 'std': std, 'min': minv, 'max': maxv, 'cv%': cv}
    else:
        n = len(arr)
        if n == 0:
            return {'n': 0, 'mean': 0, 'std': 0, 'min': 0, 'max': 0, 'cv%': 0}
        mean = sum(arr) / n
        if n == 1:
            return {'n': n, 'mean': mean, 'std': 0, 'min': arr[0], 'max': arr[0], 'cv%': 0}
        var = sum((x - mean) ** 2 for x in arr) / (n - 1)
        std = var ** 0.5
        minv = min(arr)
        maxv = max(arr)
        cv = (std / mean * 100) if mean != 0 else 0.0
        return {'n': n, 'mean': mean, 'std': std, 'min': minv, 'max': maxv, 'cv%': cv}

def main(filepath):
    cam, sys_ts, delta = load_data(filepath)
    if len(cam) < 2:
        print("Insufficient data points, cannot compute differences.")
        return

    # 1. Camera timestamp differences (frame interval)
    cam_diff = [cam[i+1] - cam[i] for i in range(len(cam)-1)]
    # 2. System timestamp differences
    sys_diff = [sys_ts[i+1] - sys_ts[i] for i in range(len(sys_ts)-1)]
    # 3. Original delta_t (latency)
    delta_vals = delta  # use the third column directly

    # Print statistics
    def print_stats(name, data, unit="ticks"):
        s = stats(data)
        print(f"--- {name} (unit: {unit}) ---")
        print(f"  Samples: {s['n']}")
        print(f"  Mean:   {s['mean']:.2f}")
        print(f"  Std Dev: {s['std']:.2f}")
        print(f"  Min:    {s['min']:.2f}")
        print(f"  Max:    {s['max']:.2f}")
        if s['cv%'] is not None:
            print(f"  CV:     {s['cv%']:.2f}%")
        print()

    print("=" * 50)
    print(f"Total frames: {len(cam)}")
    print("=" * 50)

    print_stats("Camera frame interval (cam_timestamp diff)", cam_diff)
    print_stats("System receive interval (system_time diff)", sys_diff)
    print_stats("Latency (delta_t = system_time - cam_timestamp)", delta_vals)

    # 4. Latency drift analysis: linear regression on delta_t to check for monotonic drift
    print("--- Latency drift analysis ---")
    if _HAS_NUMPY:
        x = np.arange(len(delta_vals))
        y = np.array(delta_vals, dtype=np.float64)
        A = np.vstack([x, np.ones_like(x)]).T
        m, c = np.linalg.lstsq(A, y, rcond=None)[0]
        drift_per_frame = m
        total_drift = m * (len(delta_vals) - 1)
        print(f"  Drift per frame: {drift_per_frame:.4f} ticks")
        print(f"  Total drift: {total_drift:.2f} ticks (fitted change from first to last frame)")
        # Calculate coefficient of determination R²
        y_mean = np.mean(y)
        ss_tot = np.sum((y - y_mean) ** 2)
        ss_res = np.sum((y - (m * x + c)) ** 2)
        r_squared = 1 - ss_res / ss_tot if ss_tot != 0 else 0
        print(f"  R² (goodness of fit): {r_squared:.4f}")
    else:
        # Simple head/tail difference as reference
        total_change = delta_vals[-1] - delta_vals[0]
        per_frame_change = total_change / (len(delta_vals) - 1) if len(delta_vals) > 1 else 0
        print(f"  Head-tail difference: {total_change} ticks")
        print(f"  Avg change per frame: {per_frame_change:.4f} ticks")
    print("=" * 50)

    # ========== Histogram of delta_t deviation from mean ==========
    if not _HAS_NUMPY or not _HAS_MATPLOTLIB:
        print("numpy or matplotlib not installed, cannot plot histogram.")
        return

    # Compute deviation (delta_t - mean delta_t)
    delta_mean = np.mean(delta_vals)
    offsets = np.array(delta_vals) - delta_mean

    plt.figure(figsize=(10, 6))
    plt.hist(offsets, bins=50, edgecolor='black', alpha=0.7)
    plt.axvline(x=0, color='red', linestyle='--', linewidth=1.5,
                label=f'Mean deviation = 0 (avg Δ = {delta_mean:.2f} µs)')
    plt.xlabel('delta_t deviation (µs)', fontsize=12)
    plt.ylabel('Frequency', fontsize=12)
    plt.title('Distribution of delta_t deviation from mean', fontsize=14)
    plt.grid(axis='y', alpha=0.3)
    plt.legend()
    plt.tight_layout()

    # Save the figure as PNG, naming based on input file
    import os
    base = os.path.splitext(os.path.basename(filepath))[0]
    out_img = f"{base}_delta_offset_histogram.png"
    plt.savefig(out_img, dpi=150)
    print(f"Histogram saved as: {out_img}")

    # Optionally display the plot window (may fail in headless environments, comment out if needed)
    # plt.show()

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python analyze_timestamps.py <data_file.txt>")
        sys.exit(1)
    main(sys.argv[1])