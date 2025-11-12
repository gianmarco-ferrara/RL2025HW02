from rosbags.highlevel import AnyReader
from rosbags.typesys import get_typestore, Stores
import pandas as pd
from pathlib import Path

bagpath = Path("bag_velocity_position_null_0.db3")
typestore = get_typestore(Stores.ROS2_HUMBLE)  # o ROS2_FOXY etc
records = []

with AnyReader([bagpath], default_typestore=typestore) as reader:
    # trovi la connessione che ti interessa
    for connection in reader.connections:
        if connection.topic == "/joint_states":
            for connection, timestamp, rawdata in reader.messages(
                connections=[connection]
            ):
                msg = reader.deserialize(rawdata, connection.msgtype)
                # msg è un oggetto JointState
                for i, name in enumerate(msg.name):
                    records.append(
                        {
                            "time_s": timestamp * 1e-9,
                            "joint": name,
                            "position": (
                                msg.position[i] if i < len(msg.position) else None
                            ),
                            "velocity": (
                                msg.velocity[i] if i < len(msg.velocity) else None
                            ),
                            "effort": msg.effort[i] if i < len(msg.effort) else None,
                        }
                    )

df = pd.DataFrame(records)
df.to_csv("joint_states_converted.csv", index=False)
print("Salvato CSV con", len(df), "righe")
