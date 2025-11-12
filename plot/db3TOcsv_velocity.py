from rosbags.highlevel import AnyReader
from rosbags.typesys import get_typestore, Stores
import pandas as pd
from pathlib import Path

# --- Configurazione ---
BAGFILE = Path(
    "vision_control_velocity_cmd_0.db3"
)  # Assicurati che il nome sia corretto
TOPIC_NAME = "/velocity_controller/commands"
OUTPUT_FILENAME = "velocity_commands_converted.csv"

# Definiamo i nomi dei giunti NELL'ORDINE CORRETTO
# (lo stesso ordine del tuo file /joint_states)
JOINT_NAMES = [
    "joint_a1",
    "joint_a2",
    "joint_a3",
    "joint_a4",
    "joint_a5",
    "joint_a6",
    "joint_a7",
]
# ---

# Assicurati che il file bag esista
if not BAGFILE.exists():
    print(f"Errore: File bag non trovato: {BAGFILE}")
    sys.exit(1)

typestore = get_typestore(Stores.ROS2_HUMBLE)  # o la tua versione di ROS
records = []

with AnyReader([BAGFILE], default_typestore=typestore) as reader:
    # Trova la connessione per il topic di comandi
    command_connection = None
    for connection in reader.connections:
        if connection.topic == TOPIC_NAME:
            command_connection = connection
            break

    if not command_connection:
        print(f"Errore: Topic '{TOPIC_NAME}' non trovato nel file bag.")
        sys.exit(1)

    print(f"Lettura messaggi dal topic: {TOPIC_NAME}...")

    # Itera solo sui messaggi di quel topic
    for connection, timestamp, rawdata in reader.messages(
        connections=[command_connection]
    ):
        msg = reader.deserialize(rawdata, connection.msgtype)

        # msg è un Float64MultiArray, i dati sono in 'msg.data'

        # Controlla se il numero di comandi corrisponde ai nomi dei giunti
        if len(msg.data) < len(JOINT_NAMES):
            print(
                f"Attenzione: Trovato messaggio con {len(msg.data)} comandi, ma ci aspettavamo {len(JOINT_NAMES)}. Messaggio saltato."
            )
            continue

        # Crea una riga per OGNI giunto per questo timestamp
        for i, joint_name in enumerate(JOINT_NAMES):
            records.append(
                {
                    "time_s": timestamp * 1e-9,  # Converti nanosecondi in secondi
                    "joint": joint_name,
                    "command": msg.data[
                        i
                    ],  # Questo è il valore della velocità comandata
                }
            )

# Salva nel CSV
df = pd.DataFrame(records)
df.to_csv(OUTPUT_FILENAME, index=False)
print(f"Salvato CSV '{OUTPUT_FILENAME}' con {len(df)} righe.")
