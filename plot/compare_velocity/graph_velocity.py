import pandas as pd
import matplotlib.pyplot as plt
import sys
import os
import numpy as np

FILENAME_1 = "velocity_commands_converted1.csv"
FILENAME_2 = "velocity_commands_converted2.csv"

LEGEND_1 = "velocity_ctrl (Default)"
LEGEND_2 = "velocity_ctrl_null (Null-space)"

COLOR_1 = "#960018"
COLOR_2 = "#001E47"
TIME_COL = "time_s"
JOINT_COL = "joint"
VALUE_COL = "command"

VELOCITY_THRESHOLD = 0.001

# --- MODIFICA 1: Nomi delle costanti più chiari ---
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
        # Per i comandi, ha senso considerare 'attivo' anche un comando nullo
        # Se i tuoi file partono da t=0 con comando 0, questa logica è OK
        # Se i comandi "fermi" sono NaN, .dropna() li ha già gestiti
        is_moving = (df_values.abs() > VELOCITY_THRESHOLD).any(axis=1)
        if not is_moving.any():
            print(
                f"Warning: No movement detected (values always < {VELOCITY_THRESHOLD})."
            )
            # Controlliamo se ci sono valori non-zero
            if (df_values.abs() > 0).any().any():
                print(
                    "Info: Found non-zero values, but all are below VELOCITY_THRESHOLD."
                )
                # Prendiamo il primo e ultimo tempo
                return df_values.index.min(), df_values.index.max()
            return 0, 0
        moving_times = df_values.index[is_moving]
        return moving_times.min(), moving_times.max()
    except Exception as e:
        print(f"Error while detecting motion window: {e}")
        return 0, 0


df_cmd1 = load_and_pivot_data(FILENAME_1, TIME_COL, JOINT_COL, VALUE_COL)
df_cmd2 = load_and_pivot_data(FILENAME_2, TIME_COL, JOINT_COL, VALUE_COL)

if df_cmd1 is None or df_cmd2 is None:
    print("Error loading one or both files. Exiting.")
    sys.exit(1)

print(f"Detecting motion window (based on command > {VELOCITY_THRESHOLD})...")
t_start1, t_stop1 = find_movement_window(df_cmd1)
t_start2, t_stop2 = find_movement_window(df_cmd2)

print(
    f"- File 1 ({LEGEND_1}): Motion from t_start = {t_start1:.3f}s to t_stop = {t_stop1:.3f}s"
)
print(
    f"- File 2 ({LEGEND_2}): Motion from t_start = {t_start2:.3f}s to t_stop = {t_stop2:.3f}s"
)

duration1 = t_stop1 - t_start1
duration2 = t_stop2 - t_start2
max_duration = max(duration1, duration2)

plot_start_time = 0
# --- MODIFICA 2: Calcolo del tempo di fine ---
# Aggiungiamo il padding finale per vedere il regime
plot_stop_time = PADDING_SECONDS_START + max_duration + PADDING_SECONDS_END
print(f"- Global plot window: {plot_start_time:.3f}s -> {plot_stop_time:.3f}s")
print(f"- Movements will be start-aligned at t = {PADDING_SECONDS_START:.3f}s")


joint_names1 = set(df_cmd1.columns)
joint_names2 = set(df_cmd2.columns)
all_joint_names = sorted(list(joint_names1.union(joint_names2)))

print(f"Generating {len(all_joint_names)} comparison plots...")

for joint_name in all_joint_names:
    plt.figure(figsize=(12, 6))
    ax = plt.gca()

    if joint_name in df_cmd1.columns:
        df_cmd1_filt = df_cmd1.loc[df_cmd1.index >= (t_start1 - PADDING_SECONDS_START)]

        # --- MODIFICA 3: Allineamento all'INIZIO ---
        plot_time_axis1 = df_cmd1_filt.index - t_start1 + PADDING_SECONDS_START

        # Disegna la linea piatta iniziale (pre-padding)
        first_time1 = plot_time_axis1.min()
        first_val1 = df_cmd1_filt[joint_name].iloc[0]
        if first_time1 > 0:
            ax.plot(
                [0, first_time1], [first_val1, first_val1], color=COLOR_1, linestyle="-"
            )

        # Disegna i dati principali
        ax.plot(
            plot_time_axis1,
            df_cmd1_filt[joint_name],
            label=LEGEND_1,
            color=COLOR_1,
            linestyle="-",
        )

        # --- MODIFICA 4: Estensione della linea a regime ---
        last_time1 = plot_time_axis1.max()
        if last_time1 < plot_stop_time:
            last_val1 = df_cmd1_filt[joint_name].iloc[-1]
            ax.plot(
                [last_time1, plot_stop_time],
                [last_val1, last_val1],
                color=COLOR_1,
                linestyle="-",
            )

    if joint_name in df_cmd2.columns:
        df_cmd2_filt = df_cmd2.loc[df_cmd2.index >= (t_start2 - PADDING_SECONDS_START)]

        # --- MODIFICA 3: Allineamento all'INIZIO ---
        plot_time_axis2 = df_cmd2_filt.index - t_start2 + PADDING_SECONDS_START

        # Disegna la linea piatta iniziale (pre-padding)
        first_time2 = plot_time_axis2.min()
        first_val2 = df_cmd2_filt[joint_name].iloc[0]
        if first_time2 > 0:
            ax.plot(
                [0, first_time2],
                [first_val2, first_val2],
                color=COLOR_2,
                linestyle="--",
            )

        # Disegna i dati principali
        ax.plot(
            plot_time_axis2,
            df_cmd2_filt[joint_name],
            label=LEGEND_2,
            color=COLOR_2,
            linestyle="--",
        )

        # --- MODIFICA 4: Estensione della linea a regime ---
        last_time2 = plot_time_axis2.max()
        if last_time2 < plot_stop_time:
            last_val2 = df_cmd2_filt[joint_name].iloc[-1]
            ax.plot(
                [last_time2, plot_stop_time],
                [last_val2, last_val2],
                color=COLOR_2,
                linestyle="--",
            )

    ax.set_xlim(plot_start_time, plot_stop_time)

    # --- MODIFICA 5: Titolo aggiornato ---
    ax.set_title(f"Commanded Velocity: {joint_name}", fontsize=16)

    ax.set_xlabel("Time (s)", fontsize=12)
    ax.set_ylabel("Commanded Velocity (rad/s)", fontsize=12)
    ax.grid(True)
    ax.legend()
    plt.tight_layout()

    output_filename = f"plot_compare_vel_{joint_name}.png"
    plt.savefig(output_filename)
    print(f"- Plot saved: {output_filename}")
    plt.close()

print("Completed.")
