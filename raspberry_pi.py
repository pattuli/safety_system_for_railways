import serial
import tkinter as tk
from datetime import datetime

# --- Log Storage ---
red_zone_log = [] 

# 1. Setup Serial Connection to STM32
try:
    ser = serial.Serial('/dev/serial0', 115200, timeout=1)
    serial_connected = True
except Exception as e:
    print(f"Serial Warning: {e}")
    serial_connected = False

last_status = "GREEN"  # Track previous status to detect zone entry

# 2. Main Update Loop
def update_dashboard():
    global last_status

    if serial_connected and ser.in_waiting > 0:
        try:
            line = ser.readline().decode('utf-8').strip()
            data = line.split(',')

            if len(data) == 5:
                status = data[0]
                dist = data[1]
                ir_y = data[2]
                ir_r = data[3]
                train = data[4]

                # --- Log RED zone ENTRY (only log once per entry, not every 50ms tick) ---
                if status == "RED" and last_status != "RED":
                    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                    trigger = "IR Sensor" if ir_r == "1" else "LiDAR"
                    entry = {
                        'time': timestamp,
                        'distance': dist,
                        'trigger': trigger
                    }
                    red_zone_log.append(entry)
                    refresh_log_display()

                last_status = status

                # Update Text Labels
                lbl_dist.config(text=f"LiDAR Distance: {dist} cm")
                lbl_train.config(text=f"Train at Station: {train}")
                lbl_incident_count.config(text=f"Red Zone Incidents: {len(red_zone_log)}")

                y_text = "TRIPPED (WARNING)" if ir_y == "1" else "CLEAR"
                r_text = "TRIPPED (DANGER)" if ir_r == "1" else "CLEAR"
                lbl_ir_yellow.config(text=f"Yellow Tripwire: {y_text}")
                lbl_ir_red.config(text=f"Red Tripwire: {r_text}")

                # Change Colors Based on Zone
                if status == "RED":
                    root.config(bg="#d90429")
                    lbl_status.config(text="DANGER: RED ZONE BREACH", bg="#d90429", fg="white")
                elif status == "YELLOW":
                    root.config(bg="#ffb703")
                    lbl_status.config(text="WARNING: YELLOW ZONE", bg="#ffb703", fg="black")
                elif status == "TRAIN":
                    root.config(bg="#023047")
                    lbl_status.config(text="SYSTEM IDLE: TRAIN PRESENT", bg="#023047", fg="white")
                else:
                    root.config(bg="#2a9d8f")
                    lbl_status.config(text="PLATFORM SAFE", bg="#2a9d8f", fg="white")
        except:
            pass

    root.after(50, update_dashboard)

def refresh_log_display():
    """Rebuild the log listbox from red_zone_log."""
    log_listbox.delete(0, tk.END)
    for i, entry in enumerate(red_zone_log):
        log_listbox.insert(
            tk.END,
            f"#{i+1}  |  {entry['time']}  |  {entry['distance']} cm  |  via {entry['trigger']}"
        )
    # Auto-scroll to latest entry
    if red_zone_log:
        log_listbox.see(tk.END)

def clear_log():
    """Clear all log entries."""
    red_zone_log.clear()
    refresh_log_display()
    lbl_incident_count.config(text="Red Zone Incidents: 0")

def save_log():
    """Save log to a text file."""
    filename = f"red_zone_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"
    with open(filename, 'w') as f:
        f.write("Railway Platform - Red Zone Incident Log\n")
        f.write("=" * 60 + "\n")
        for i, entry in enumerate(red_zone_log):
            f.write(f"#{i+1} | {entry['time']} | {entry['distance']} cm | via {entry['trigger']}\n")
        f.write("=" * 60 + "\n")
        f.write(f"Total Incidents: {len(red_zone_log)}\n")
    lbl_save_confirm.config(text=f"Saved: {filename}")
    root.after(3000, lambda: lbl_save_confirm.config(text=""))  # Clear message after 3s

# 3. Build the User Interface
root = tk.Tk()
root.title("Railway Platform Safety Control Center")
root.geometry("750x750")
root.config(bg="#333333")

# --- Top Status Banner ---
lbl_status = tk.Label(root, text="WAITING FOR STM32 DATA...", font=("Arial", 24, "bold"),
                       bg="#333333", fg="white", pady=20)
lbl_status.pack(fill='x')

# --- Live Sensor Diagnostics ---
data_frame = tk.Frame(root, bg="white", bd=4, relief="ridge")
data_frame.pack(pady=10, padx=40, fill='x')

tk.Label(data_frame, text="--- LIVE SENSOR DIAGNOSTICS ---", font=("Arial", 13, "bold"),
         bg="white", pady=8).pack()
lbl_dist = tk.Label(data_frame, text="LiDAR Distance: --", font=("Arial", 15), bg="white")
lbl_dist.pack(pady=3)
lbl_ir_yellow = tk.Label(data_frame, text="Yellow Tripwire: --", font=("Arial", 15), bg="white")
lbl_ir_yellow.pack(pady=3)
lbl_ir_red = tk.Label(data_frame, text="Red Tripwire: --", font=("Arial", 15), bg="white")
lbl_ir_red.pack(pady=3)
lbl_train = tk.Label(data_frame, text="Train at Station: --", font=("Arial", 15), bg="white")
lbl_train.pack(pady=3)

# --- Incident Counter ---
lbl_incident_count = tk.Label(root, text="Red Zone Incidents: 0",
                               font=("Arial", 15, "bold"), bg="#333333", fg="#ff6b6b")
lbl_incident_count.pack(pady=5)

# --- Red Zone Log Panel ---
log_frame = tk.Frame(root, bg="#1a1a2e", bd=4, relief="ridge")
log_frame.pack(pady=5, padx=40, fill='both', expand=True)

tk.Label(log_frame, text="RED ZONE INCIDENT LOG", font=("Arial", 12, "bold"),
         bg="#1a1a2e", fg="#ffffff", pady=6).pack()

# Scrollable listbox
scrollbar = tk.Scrollbar(log_frame)
scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

log_listbox = tk.Listbox(log_frame, yscrollcommand=scrollbar.set,
                          font=("Courier", 11), bg="#0d0d0d", fg="#ffffff",
                          selectbackground="#d90429", height=10)
log_listbox.pack(fill='both', expand=True, padx=5, pady=5)
scrollbar.config(command=log_listbox.yview)

# --- Buttons ---
btn_frame = tk.Frame(root, bg="#333333")
btn_frame.pack(pady=8)

tk.Button(btn_frame, text="Save Log to File", font=("Arial", 12),
          bg="#457b9d", fg="white", padx=10, command=save_log).pack(side=tk.LEFT, padx=10)

tk.Button(btn_frame, text="Clear Log", font=("Arial", 12),
          bg="#6c757d", fg="white", padx=10, command=clear_log).pack(side=tk.LEFT, padx=10)

# Save confirmation message
lbl_save_confirm = tk.Label(root, text="", font=("Arial", 10), bg="#333333", fg="#90ee90")
lbl_save_confirm.pack()

# Start the loop
update_dashboard()
root.mainloop()
