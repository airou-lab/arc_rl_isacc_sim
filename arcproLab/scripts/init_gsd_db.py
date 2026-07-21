import sqlite3
import os

db_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../.gsd/archive.db"))
os.makedirs(os.path.dirname(db_path), exist_ok=True)

conn = sqlite3.connect(db_path)
c = conn.cursor()
# Create table for events
c.execute('''CREATE TABLE IF NOT EXISTS telemetry_events (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
    event_type TEXT,
    metrics JSON,
    notes TEXT
)''')
conn.commit()
conn.close()
print(f"Initialized lightweight GSD database at {db_path}")
