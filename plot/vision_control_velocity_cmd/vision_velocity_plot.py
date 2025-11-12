import pandas as pd
import matplotlib.pyplot as plt
import sys
import os
import numpy as np

# --- MODIFICA 1: Impostazioni per file singolo ---
FILENAME = "velocity_commands_converted.csv"  # <-- Inserisci qui il tuo file
LEGEND = "velocity_cmd"  # <-- Inserisci qui la legenda
COLOR = "#960018"  # <-- Inserisci qui il colore

TIME_COL = "time_s"
JOINT_COL = "joint"
VALUE_COL = "command"

VELOCITY_THRESHOLD = 0.001
PADDING_SECONDS_START = 4.0  # Padding *prima* dell'inizio
PADDING_SECONDS_END = 5.0  # Padding *dopo* la fine (per vedere il regime)


def load_and_pivot_data(filename, time_col, joint_col, value_col):
    print(f"Loading file: {filename}")
    if not os.path.exists(filename):
        print(f"Error: File not found: {filename}")
        return None

    try:
        df = pd.read_csv(filename)
    except Exception as e:
        print(f"Error reading CSV '{filename}': {e}")
        return None

    required_cols = [time_col, joint_col, value_col]
    if not all(col in df.columns for col in required_cols):
        print(f"Error: CSV '{filename}' must contain columns: {required_cols}")
        print(f"Columns found: {list(df.columns)}")
        return None

    print(f"Transforming data (Pivot) for {filename}...")
    try:
        df_wide = df.pivot(index=time_col, columns=joint_col, values=value_col)
    except Exception as e:
        print(f"Error during pivot for '{filename}': {e}")
        print("Check for duplicate timestamps per joint.")
        return None

    print(f"Normalizing time for {filename}...")
    try:
        df_wide.index = pd.to_numeric(df_wide.index, errors="coerce")
        df_wide = df_wide.dropna(axis=0, how="any")
        df_wide.index = df_wide.index - df_wide.index[0]
    except Exception as e:
        print(f"Error during time normalization for '{filename}': {e}")
        return None

    return df_wide


def find_movement_window(df_values):
    try:
        is_moving = (df_values.abs() > VELOCITY_THRESHOLD).any(axis=1)
        if not is_moving.any():
            print(
                f"Warning: No movement detected (values always < {VELOCITY_THRESHOLD})."
            )
            if (df_values.abs() > 0).any().any():
                print(
                    "Info: Found non-zero values, but all are below VELOCITY_THRESHOLD."
                )
                return df_values.index.min(), df_values.index.max()
            return 0, 0
        moving_times = df_values.index[is_moving]
        return moving_times.min(), moving_times.max()
    except Exception as e:
        print(f"Error while detecting motion window: {e}")
        return 0, 0


# --- MODIFICA 2: Caricamento singolo ---
df_cmd = load_and_pivot_data(FILENAME, TIME_COL, JOINT_COL, VALUE_COL)

if df_cmd is None:
    print("Error loading file. Exiting.")
    sys.exit(1)

print(f"Detecting motion window (based on command > {VELOCITY_THRESHOLD})...")
t_start, t_stop = find_movement_window(df_cmd)

print(
    f"- File ({LEGEND}): Motion from t_start = {t_start:.3f}s to t_stop = {t_stop:.3f}s"
)

# --- MODIFICA 3: Calcolo durata singola ---
duration = t_stop - t_start
plot_start_time = 0
plot_stop_time = PADDING_SECONDS_START + duration + PADDING_SECONDS_END

print(f"- Global plot window: {plot_start_time:.3f}s -> {plot_stop_time:.3f}s")
print(f"- Movement will be start-aligned at t = {PADDING_SECONDS_START:.3f}s")

# --- MODIFICA 4: Lista joint singola ---
all_joint_names = sorted(list(df_cmd.columns))

print(f"Generating {len(all_joint_names)} plots...")

for joint_name in all_joint_names:
    plt.figure(figsize=(12, 6))
    ax = plt.gca()

    # --- MODIFICA 5: Logica di plot singola ---
    df_cmd_filt = df_cmd.loc[df_cmd.index >= (t_start - PADDING_SECONDS_START)]

    plot_time_axis = df_cmd_filt.index - t_start + PADDING_SECONDS_START

    # Disegna la linea piatta iniziale (pre-padding)
    first_time = plot_time_axis.min()
    first_val = df_cmd_filt[joint_name].iloc[0]
    if first_time > 0:
        ax.plot([0, first_time], [first_val, first_val], color=COLOR, linestyle="-")

    # Disegna i dati principali
    ax.plot(
        plot_time_axis,
        df_cmd_filt[joint_name],
        label=LEGEND,
        color=COLOR,
        linestyle="-",
    )

    # Estensione della linea a regime
    last_time = plot_time_axis.max()
    if last_time < plot_stop_time:
        last_val = df_cmd_filt[joint_name].iloc[-1]
        ax.plot(
            [last_time, plot_stop_time],
            [last_val, last_val],
            color=COLOR,
            linestyle="-",
        )

    ax.set_xlim(plot_start_time, plot_stop_time)
    ax.set_title(f"Commanded Velocity: {joint_name}", fontsize=16)
    ax.set_xlabel("Time (s)", fontsize=12)
    ax.set_ylabel("Commanded Velocity (rad/s)", fontsize=12)
    ax.grid(True)
    ax.legend()
    plt.tight_layout()

    # --- MODIFICA 6: Nome file di output ---
    output_filename = f"plot_vel_{joint_name}.png"
    plt.savefig(output_filename)
    print(f"- Plot saved: {output_filename}")
    plt.close()

print("Completed.")
