import tkinter as tk


class Dashboard:
    def __init__(self):
        self.win = tk.Tk()
        self.win.title("Dashboard")
        self.label_acc = tk.Label(self.win, text="Current Acceleration: ")
        self.text_acc = tk.Entry(self.win, width=15)
        self.label_speed = tk.Label(self.win, text="Current Speed: ")
        self.text_speed = tk.Entry(self.win, width=15)
        self.label_dir = tk.Label(self.win, text="Current Orientation: ")
        self.text_dir = tk.Entry(self.win, width=15)
        self.label_conflict = tk.Label(self.win, text="Current Conflict Vehicle ID: ")
        self.text_conflict = tk.Entry(self.win, width=15)
        self.btn_detected = tk.Button(self.win, width=12, text="No Conflict", bg='green')
        self.label_spacer1 = tk.Label(self.win, width=15, text="")
        self.label_spacer2 = tk.Label(self.win, width=15, text="")

        label_list = [self.label_acc, self.text_acc, self.label_speed, self.text_speed, self.label_dir, self.text_dir,
                      self.label_conflict, self.text_conflict, self.label_spacer1, self.btn_detected, self.label_spacer2]
        for label in label_list:
            label.pack()
        self.win.update()

    def set_text(self, entry, text):
        entry.delete(0, tk.END)
        entry.insert(0, text)
        self.win.update()
        return

    def update_dash(self, agent, activator):
        self.set_text(self.text_acc, str(activator.current_acceleration))
        self.set_text(self.text_speed, str(agent.get_speed()[0]))
        self.set_text(self.text_dir, str(agent.angle))
        self.update_detection_indicator(activator)

    def update_detection_indicator(self, activator):
        if activator.current_conflict_vehicle:
            self.set_text(self.text_conflict, activator.current_conflict_vehicle)
            self.btn_detected.config(text="Conflict detected!", bg='red')
        else:
            self.set_text(self.text_conflict, "NONE")
            self.btn_detected.config(text="No Conflict", bg='green')
        self.win.update()
