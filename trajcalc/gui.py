"""Tkinter GUI front-end, launched automatically when run with no CLI arguments.

Lets the user load two photos, click the missile's position in each, and
Calculate — using sensible defaults for any parameter left blank. An optional
"stereo" mode adds a second camera + calibration file for accurate 3D
triangulation; without it, a single-camera best-effort approximation is used
(see trajcalc.monocular).
"""
from __future__ import annotations

import tkinter as tk
from tkinter import filedialog, messagebox, ttk

from PIL import Image, ImageTk

from trajcalc import ballistics, calibration, monocular, triangulate, visualize

CANVAS_MAX_DIM = 380

DEFAULTS = {
    "dt": 0.5,
    "mass": 10.0,
    "area": 0.05,
    "drag_coeff": 0.3,
    "air_density": 1.225,
    "ground_z": 0.0,
    "distance": 100.0,
    "fov_deg": 60.0,
    "camera_height": 1.5,
}


class ImagePicker(ttk.Frame):
    """A labeled image loader + click-to-mark canvas."""

    def __init__(self, parent, title):
        super().__init__(parent, padding=6, relief="groove", borderwidth=1)
        self.image_path = None
        self.original_size = (0, 0)
        self.scale = 1.0
        self.pixel = None
        self._tk_image = None

        ttk.Label(self, text=title, font=("", 10, "bold")).pack(anchor="w")
        ttk.Button(self, text="Browse...", command=self._browse).pack(anchor="w", pady=(2, 4))

        self.canvas = tk.Canvas(self, width=CANVAS_MAX_DIM, height=CANVAS_MAX_DIM // 4 * 3, bg="#222222")
        self.canvas.pack()
        self.canvas.bind("<Button-1>", self._on_click)

        self.status = ttk.Label(self, text="No image loaded", wraplength=CANVAS_MAX_DIM)
        self.status.pack(anchor="w", pady=(4, 0))

    def _browse(self):
        path = filedialog.askopenfilename(
            title="Select photo",
            filetypes=[("Images", "*.jpg *.jpeg *.png *.bmp"), ("All files", "*.*")],
        )
        if path:
            self.load(path)

    def load(self, path):
        image = Image.open(path)
        self.image_path = path
        self.original_size = image.size
        self.pixel = None

        w, h = image.size
        scale = min(CANVAS_MAX_DIM / w, CANVAS_MAX_DIM / h, 1.0)
        self.scale = scale
        display = image.resize((max(1, int(w * scale)), max(1, int(h * scale))))

        self._tk_image = ImageTk.PhotoImage(display)
        self.canvas.config(width=display.width, height=display.height)
        self.canvas.delete("all")
        self.canvas.create_image(0, 0, anchor="nw", image=self._tk_image)
        name = path.replace("\\", "/").rsplit("/", 1)[-1]
        self.status.config(text=f"{name}  ({w}x{h}) — click the missile's position")

    def _on_click(self, event):
        if self.image_path is None:
            return
        self.pixel = (event.x / self.scale, event.y / self.scale)
        self.canvas.delete("mark")
        r = 6
        self.canvas.create_oval(
            event.x - r, event.y - r, event.x + r, event.y + r, outline="red", width=2, tags="mark"
        )
        self.status.config(text=f"Marked at pixel ({self.pixel[0]:.0f}, {self.pixel[1]:.0f})")

    def is_ready(self) -> bool:
        return self.image_path is not None and self.pixel is not None


def _labeled_entry(parent, label, default):
    row = ttk.Frame(parent)
    row.pack(fill="x", pady=2)
    ttk.Label(row, text=label, width=34).pack(side="left")
    var = tk.StringVar(value=str(default))
    ttk.Entry(row, textvariable=var, width=10).pack(side="left")
    return var


def _read_float(var, default, name, assumptions):
    text = var.get().strip()
    if not text:
        assumptions.append(f"{name}: no value given, used default {default}")
        return default
    try:
        return float(text)
    except ValueError:
        assumptions.append(f"{name}: '{text}' isn't a number, used default {default}")
        return default


class App(ttk.Frame):
    def __init__(self, root):
        super().__init__(root, padding=10)
        self.pack(fill="both", expand=True)
        root.title("Missile Trajectory Calculator (ALPHA)")

        self.cameras = None

        ttk.Label(
            self,
            text="Load a photo of the missile at two moments, click its position in each, "
            "then Calculate. Leave any parameter blank to use a reasonable default.",
            wraplength=780,
        ).pack(anchor="w", pady=(0, 8))

        images_row = ttk.Frame(self)
        images_row.pack(fill="x")
        self.picker_t1 = ImagePicker(images_row, "Point 1 (earlier)")
        self.picker_t1.pack(side="left", padx=4)
        self.picker_t2 = ImagePicker(images_row, "Point 2 (later)")
        self.picker_t2.pack(side="left", padx=4)

        self.stereo_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(
            self,
            text="Advanced: I have a second camera + calibration file (accurate 3D triangulation)",
            variable=self.stereo_var,
            command=self._toggle_stereo,
        ).pack(anchor="w", pady=(8, 0))

        self.stereo_frame = ttk.Frame(self)
        cal_row = ttk.Frame(self.stereo_frame)
        cal_row.pack(fill="x", pady=4)
        ttk.Button(cal_row, text="Load calibration.yaml...", command=self._load_calibration).pack(side="left")
        self.calibration_label = ttk.Label(cal_row, text="No calibration loaded")
        self.calibration_label.pack(side="left", padx=8)

        images_row_b = ttk.Frame(self.stereo_frame)
        images_row_b.pack(fill="x")
        self.picker_b_t1 = ImagePicker(images_row_b, "Point 1 (Camera B)")
        self.picker_b_t1.pack(side="left", padx=4)
        self.picker_b_t2 = ImagePicker(images_row_b, "Point 2 (Camera B)")
        self.picker_b_t2.pack(side="left", padx=4)

        params = ttk.LabelFrame(self, text="Parameters (defaults used if left blank)")
        params.pack(fill="x", pady=10)
        left = ttk.Frame(params)
        left.pack(side="left", padx=10, pady=6)
        right = ttk.Frame(params)
        right.pack(side="left", padx=10, pady=6)

        self.var_dt = _labeled_entry(left, "Time between photos (s)", DEFAULTS["dt"])
        self.var_mass = _labeled_entry(left, "Mass (kg)", DEFAULTS["mass"])
        self.var_area = _labeled_entry(left, "Cross-sectional area (m^2)", DEFAULTS["area"])
        self.var_drag = _labeled_entry(left, "Drag coefficient", DEFAULTS["drag_coeff"])
        self.var_density = _labeled_entry(right, "Air density (kg/m^3)", DEFAULTS["air_density"])
        self.var_ground_z = _labeled_entry(right, "Ground elevation (m)", DEFAULTS["ground_z"])
        self.var_distance = _labeled_entry(
            right, "Monocular: assumed distance to missile (m)", DEFAULTS["distance"]
        )
        self.var_fov = _labeled_entry(right, "Monocular: camera horizontal FOV (deg)", DEFAULTS["fov_deg"])
        self.var_camera_height = _labeled_entry(
            right, "Monocular: camera height above ground (m)", DEFAULTS["camera_height"]
        )

        ttk.Button(self, text="Calculate", command=self._calculate).pack(pady=8)

        self.output = tk.Text(self, height=10, width=95, wrap="word")
        self.output.pack(fill="both", expand=False, pady=(4, 0))

        self.plot_frame = ttk.Frame(self)
        self.plot_frame.pack(fill="both", expand=True, pady=(8, 0))

    def _toggle_stereo(self):
        if self.stereo_var.get():
            self.stereo_frame.pack(fill="x", pady=4)
        else:
            self.stereo_frame.pack_forget()

    def _load_calibration(self):
        path = filedialog.askopenfilename(
            title="Select calibration YAML", filetypes=[("YAML", "*.yaml *.yml"), ("All files", "*.*")]
        )
        if not path:
            return
        try:
            self.cameras = calibration.load_calibration(path)
            name = path.replace("\\", "/").rsplit("/", 1)[-1]
            self.calibration_label.config(text=name)
        except Exception as exc:
            messagebox.showerror("Calibration error", str(exc))

    def _calculate(self):
        assumptions = []

        if not self.picker_t1.is_ready() or not self.picker_t2.is_ready():
            messagebox.showwarning(
                "Missing input", "Load both photos and click the missile's position in each first."
            )
            return

        dt = _read_float(self.var_dt, DEFAULTS["dt"], "Time between photos", assumptions)
        mass = _read_float(self.var_mass, DEFAULTS["mass"], "Mass", assumptions)
        area = _read_float(self.var_area, DEFAULTS["area"], "Cross-sectional area", assumptions)
        drag_coeff = _read_float(self.var_drag, DEFAULTS["drag_coeff"], "Drag coefficient", assumptions)
        air_density = _read_float(self.var_density, DEFAULTS["air_density"], "Air density", assumptions)
        ground_z = _read_float(self.var_ground_z, DEFAULTS["ground_z"], "Ground elevation", assumptions)

        use_stereo = (
            self.stereo_var.get()
            and self.cameras is not None
            and self.picker_b_t1.is_ready()
            and self.picker_b_t2.is_ready()
        )

        try:
            if use_stereo:
                cam_a, cam_b = self.cameras["A"], self.cameras["B"]
                p1 = triangulate.triangulate_point(cam_a, cam_b, self.picker_t1.pixel, self.picker_b_t1.pixel)
                p2 = triangulate.triangulate_point(cam_a, cam_b, self.picker_t2.pixel, self.picker_b_t2.pixel)
            else:
                if self.stereo_var.get():
                    assumptions.append(
                        "Stereo mode was checked but the calibration file or second camera's "
                        "photos weren't provided — fell back to single-camera approximation."
                    )
                distance = _read_float(
                    self.var_distance, DEFAULTS["distance"], "Assumed distance to missile", assumptions
                )
                fov_deg = _read_float(self.var_fov, DEFAULTS["fov_deg"], "Camera FOV", assumptions)
                camera_height = _read_float(
                    self.var_camera_height, DEFAULTS["camera_height"], "Camera height", assumptions
                )
                assumptions.append(
                    f"Single-camera mode: assumes a level camera aimed horizontally, "
                    f"{camera_height:.1f} m above the ground, with the missile at a roughly "
                    f"constant distance of {distance:.0f} m — an approximation, not a measurement."
                )
                w, h = self.picker_t1.original_size
                cam = monocular.default_camera(w, h, fov_deg=fov_deg, camera_height=camera_height)
                p1 = monocular.pixel_to_point_at_distance(cam, self.picker_t1.pixel, distance)
                p2 = monocular.pixel_to_point_at_distance(cam, self.picker_t2.pixel, distance)

            k = ballistics.drag_k(mass, area, drag_coeff, air_density)
            v0, result = ballistics.solve_initial_velocity(p1, p2, dt, k)
            if not result.success:
                assumptions.append(f"Velocity solver did not fully converge ({result.message})")

            trajectory = ballistics.simulate_trajectory(p1, v0, k, dt=0.001, t_max=300.0, ground_z=ground_z)
        except Exception as exc:
            messagebox.showerror("Calculation error", str(exc))
            return

        report = visualize.summarize(p1, v0, trajectory)
        if assumptions:
            report += "\n\nAssumptions used:\n" + "\n".join(f"- {a}" for a in assumptions)

        self.output.delete("1.0", "end")
        self.output.insert("1.0", report)

        self._show_plot(p1, p2, trajectory)

    def _show_plot(self, p1, p2, trajectory):
        import matplotlib.pyplot as plt
        from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

        for child in self.plot_frame.winfo_children():
            child.destroy()

        positions = trajectory["positions"]
        fig = plt.Figure(figsize=(7, 5))
        ax = fig.add_subplot(111, projection="3d")
        ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], label="Fitted trajectory")
        ax.scatter(*p1, color="green", s=50, label="Point 1")
        ax.scatter(*p2, color="orange", s=50, label="Point 2")
        _, apogee_pos = trajectory["apogee"]
        ax.scatter(*apogee_pos, color="red", marker="^", s=70, label="Apogee")
        if trajectory["impact"] is not None:
            _, impact_pos = trajectory["impact"]
            ax.scatter(*impact_pos, color="black", marker="x", s=70, label="Impact")
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_zlabel("Z (m)")
        ax.legend()

        canvas = FigureCanvasTkAgg(fig, master=self.plot_frame)
        canvas.draw()
        canvas.get_tk_widget().pack(fill="both", expand=True)


def launch():
    root = tk.Tk()
    App(root)
    root.mainloop()
