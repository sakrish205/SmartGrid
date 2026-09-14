"""
build_installer.py — SmartGrid developer build tool
Run: python build_installer.py
Output: %USERPROFILE%\Desktop\SmartGrid_Setup.exe
"""
import base64
import os
import pathlib
import subprocess
import sys
import zipfile

ROOT  = pathlib.Path(__file__).parent.resolve()
BUILD = ROOT / 'build'
SKIP  = {'.git', '__pycache__', 'build', 'dist', '.pytest_cache'}

# ---------------------------------------------------------------------------
# INSTALLER_TEMPLATE — the tkinter wizard baked into SmartGrid_Setup.exe
# __EMBEDDED_ZIP__ is replaced at build time with the base64-encoded project zip
# ---------------------------------------------------------------------------
INSTALLER_TEMPLATE = r'''
import base64, io, os, pathlib, shutil, subprocess, sys, threading, urllib.request, zipfile
import tkinter as tk
from tkinter import filedialog, font, scrolledtext, ttk

EMBEDDED_ZIP = "__EMBEDDED_ZIP__"

APP_NAME = "SmartGrid"
DEFAULT_INSTALL = str(pathlib.Path.home() / "AppData" / "Local" / APP_NAME)
PYTHON_VERSION = (3, 12)
PYTHON_WINGET_ID = "Python.Python.3.12"
PYTHON_INSTALLER_URL = "https://www.python.org/ftp/python/3.12.10/python-3.12.10-amd64.exe"


def _find_python():
    for cmd in ("python", "python3", "py"):
        exe = shutil.which(cmd)
        if not exe:
            continue
        try:
            raw = subprocess.check_output([exe, "--version"],
                                          stderr=subprocess.STDOUT).decode().strip()
            parts = raw.split()[1].split(".")
            ver = tuple(int(x) for x in parts[:2])
            if ver >= PYTHON_VERSION:
                return exe
        except Exception:
            continue
    return None


def _install_python_winget():
    r = subprocess.run(
        ["winget", "install", "--id", PYTHON_WINGET_ID,
         "--silent", "--accept-package-agreements", "--accept-source-agreements"],
        capture_output=True)
    return r.returncode == 0


def _install_python_web(log):
    tmp = os.path.join(os.environ.get("TEMP", os.getcwd()), "python_installer.exe")
    log(f"  Downloading Python from python.org …")
    urllib.request.urlretrieve(PYTHON_INSTALLER_URL, tmp)
    subprocess.run([tmp, "/quiet", "InstallAllUsers=0",
                    "PrependPath=1", "Include_pip=1"], check=True)


def _create_shortcut(python_exe, install_dir):
    pythonw = python_exe.replace("python.exe", "pythonw.exe")
    if not os.path.exists(pythonw):
        pythonw = python_exe
    main_py = os.path.join(install_dir, "main.py")
    desktop = os.path.join(os.path.expanduser("~"), "Desktop")
    lnk = os.path.join(desktop, f"{APP_NAME}.lnk")
    ps = (
        f'$ws=(New-Object -com WScript.Shell);'
        f'$s=$ws.CreateShortcut("{lnk}");'
        f'$s.TargetPath="{pythonw}";'
        f'$s.Arguments=\'"{main_py}"\';'
        f'$s.WorkingDirectory="{install_dir}";'
        f'$s.Description="{APP_NAME} — 3D Spray Path Generator";'
        f'$s.Save()'
    )
    subprocess.run(["powershell", "-Command", ps], check=True)


# ---------------------------------------------------------------------------
# GUI
# ---------------------------------------------------------------------------
class Wizard(tk.Tk):
    BG     = "#ffffff"
    HEADER = "#0078D4"
    FG     = "#252525"
    BTN_FG = "#ffffff"

    def __init__(self):
        super().__init__()
        self.title(f"{APP_NAME} Setup")
        self.resizable(False, False)
        self.configure(bg=self.BG)
        w, h = 560, 400
        sw, sh = self.winfo_screenwidth(), self.winfo_screenheight()
        self.geometry(f"{w}x{h}+{(sw-w)//2}+{(sh-h)//2}")

        self._install_dir = tk.StringVar(value=DEFAULT_INSTALL)
        self._python_exe  = None
        self._frames      = {}

        self._build_header()
        self._container = tk.Frame(self, bg=self.BG)
        self._container.pack(fill="both", expand=True, padx=24, pady=(0, 8))
        self._build_nav()

        for F in (WelcomePage, LocationPage, InstallingPage, FinishPage):
            frame = F(self._container, self)
            self._frames[F.__name__] = frame
            frame.place(relwidth=1, relheight=1)

        self.show("WelcomePage")

    def _build_header(self):
        hdr = tk.Frame(self, bg=self.HEADER, height=56)
        hdr.pack(fill="x")
        hdr.pack_propagate(False)
        tk.Label(hdr, text=f"  {APP_NAME} Setup", bg=self.HEADER, fg=self.BTN_FG,
                 font=("Segoe UI", 14, "bold"), anchor="w").pack(
            side="left", fill="y", padx=8)

    def _build_nav(self):
        nav = tk.Frame(self, bg="#f3f2f1", height=48)
        nav.pack(fill="x", side="bottom")
        nav.pack_propagate(False)
        self._btn_back = tk.Button(nav, text="‹ Back", width=10,
                                   bg="#ffffff", fg=self.FG, relief="flat",
                                   bd=1, highlightbackground="#d2d0ce",
                                   command=self._back)
        self._btn_next = tk.Button(nav, text="Next ›", width=10,
                                   bg=self.HEADER, fg=self.BTN_FG, relief="flat",
                                   command=self._next)
        self._btn_cancel = tk.Button(nav, text="Cancel", width=10,
                                     bg="#ffffff", fg=self.FG, relief="flat",
                                     bd=1, highlightbackground="#d2d0ce",
                                     command=self.destroy)
        self._btn_cancel.pack(side="right", padx=8, pady=8)
        self._btn_next.pack(side="right", padx=4, pady=8)
        self._btn_back.pack(side="right", padx=0, pady=8)
        self._page_order = ["WelcomePage", "LocationPage", "InstallingPage", "FinishPage"]
        self._current = 0

    def show(self, name):
        self._frames[name].tkraise()
        idx = self._page_order.index(name)
        self._btn_back.config(state="normal" if idx > 0 else "disabled")
        if name == "FinishPage":
            self._btn_next.config(state="disabled")
            self._btn_back.config(state="disabled")
        elif name == "InstallingPage":
            self._btn_next.config(state="disabled")
            self._btn_back.config(state="disabled")

    def _next(self):
        self._current = min(self._current + 1, len(self._page_order) - 1)
        name = self._page_order[self._current]
        self.show(name)
        if name == "InstallingPage":
            self._frames["InstallingPage"].start(
                self._install_dir.get(), self._on_install_done)

    def _back(self):
        self._current = max(self._current - 1, 0)
        self.show(self._page_order[self._current])

    def _on_install_done(self, python_exe):
        self._python_exe = python_exe
        self._btn_next.config(state="normal")
        self._current = len(self._page_order) - 1
        self.show("FinishPage")
        self._frames["FinishPage"].set_install_dir(self._install_dir.get(), python_exe)


# ---------------------------------------------------------------------------
# Pages
# ---------------------------------------------------------------------------
class WelcomePage(tk.Frame):
    def __init__(self, parent, wiz):
        super().__init__(parent, bg=wiz.BG)
        tk.Label(self, text=f"Welcome to {APP_NAME} Setup",
                 bg=wiz.BG, fg=wiz.FG, font=("Segoe UI", 13, "bold")).pack(
            anchor="w", pady=(20, 6))
        tk.Label(self, text=(
            "This wizard will install SmartGrid — a 3D robotic spray-paint\n"
            "toolpath planner — on your computer.\n\n"
            "It will:\n"
            "  • Check for Python 3.12 (and install it if missing)\n"
            "  • Extract SmartGrid to a folder you choose\n"
            "  • Install all required Python packages\n"
            "  • Create a desktop shortcut\n\n"
            "Click Next to continue."
        ), bg=wiz.BG, fg=wiz.FG, font=("Segoe UI", 10), justify="left").pack(
            anchor="w", padx=4)


class LocationPage(tk.Frame):
    def __init__(self, parent, wiz):
        super().__init__(parent, bg=wiz.BG)
        self._wiz = wiz
        tk.Label(self, text="Install Location",
                 bg=wiz.BG, fg=wiz.FG, font=("Segoe UI", 13, "bold")).pack(
            anchor="w", pady=(20, 4))
        tk.Label(self, text="Choose where to install SmartGrid:",
                 bg=wiz.BG, fg=wiz.FG, font=("Segoe UI", 10)).pack(
            anchor="w", pady=(0, 6))
        row = tk.Frame(self, bg=wiz.BG)
        row.pack(fill="x", pady=4)
        tk.Entry(row, textvariable=wiz._install_dir, width=48,
                 font=("Segoe UI", 10), relief="solid", bd=1).pack(
            side="left", padx=(0, 4))
        tk.Button(row, text="Browse…", bg="#ffffff", fg=wiz.FG, relief="flat",
                  bd=1, highlightbackground="#d2d0ce",
                  command=self._browse).pack(side="left")
        tk.Label(self, text=(
            "Requires ~400 MB disk space.\n"
            "Python packages will be installed to your local Python environment."
        ), bg=wiz.BG, fg="#605e5c", font=("Segoe UI", 9), justify="left").pack(
            anchor="w", pady=(12, 0))

    def _browse(self):
        d = filedialog.askdirectory(title="Choose install folder")
        if d:
            self._wiz._install_dir.set(d.replace("/", os.sep))


class InstallingPage(tk.Frame):
    def __init__(self, parent, wiz):
        super().__init__(parent, bg=wiz.BG)
        self._wiz = wiz
        tk.Label(self, text="Installing…",
                 bg=wiz.BG, fg=wiz.FG, font=("Segoe UI", 13, "bold")).pack(
            anchor="w", pady=(16, 4))
        self._bar = ttk.Progressbar(self, mode="indeterminate", length=480)
        self._bar.pack(anchor="w", pady=(0, 6))
        self._log = scrolledtext.ScrolledText(
            self, height=11, font=("Consolas", 9),
            bg="#fafafa", fg="#252525", relief="solid", bd=1, state="disabled")
        self._log.pack(fill="both", expand=True)

    def _append(self, text):
        self._log.config(state="normal")
        self._log.insert("end", text + "\n")
        self._log.see("end")
        self._log.config(state="disabled")

    def start(self, install_dir, done_cb):
        self._bar.start(12)
        threading.Thread(target=self._run, args=(install_dir, done_cb),
                         daemon=True).start()

    def _run(self, install_dir, done_cb):
        log = self._append
        python_exe = None
        try:
            # --- Step 1: Python ---
            log("Step 1/4  Checking Python 3.12 …")
            python_exe = _find_python()
            if python_exe:
                log(f"  Found: {python_exe}  ✓")
            else:
                log("  Not found — installing via winget …")
                ok = _install_python_winget()
                if not ok:
                    log("  winget failed — downloading from python.org …")
                    _install_python_web(log)
                # search again after install
                python_exe = _find_python()
                if not python_exe:
                    raise RuntimeError(
                        "Python 3.12 installation failed. "
                        "Install it manually from python.org and re-run Setup.")
                log(f"  Installed: {python_exe}  ✓")

            # --- Step 2: Extract ---
            log("Step 2/4  Extracting SmartGrid …")
            zip_bytes = base64.b64decode(EMBEDDED_ZIP)
            if os.path.exists(install_dir):
                shutil.rmtree(install_dir)
            os.makedirs(install_dir, exist_ok=True)
            with zipfile.ZipFile(io.BytesIO(zip_bytes)) as zf:
                zf.extractall(install_dir)
            log(f"  Extracted to {install_dir}  ✓")

            # --- Step 3: pip ---
            log("Step 3/4  Installing requirements (this may take a few minutes) …")
            req = os.path.join(install_dir, "requirements.txt")
            proc = subprocess.Popen(
                [python_exe, "-m", "pip", "install", "-r", req,
                 "--no-warn-script-location"],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, encoding="utf-8", errors="replace")
            for line in proc.stdout:
                log("  " + line.rstrip())
            proc.wait()
            if proc.returncode != 0:
                raise RuntimeError("pip install failed — see log above.")
            log("  Requirements installed  ✓")

            # --- Step 4: Shortcut ---
            log("Step 4/4  Creating desktop shortcut …")
            _create_shortcut(python_exe, install_dir)
            log("  Desktop shortcut created  ✓")

            log("\n✓  Installation complete!")
        except Exception as exc:
            log(f"\n✗  Error: {exc}")
            python_exe = None
        finally:
            self._bar.stop()
            self.after(0, done_cb, python_exe)


class FinishPage(tk.Frame):
    def __init__(self, parent, wiz):
        super().__init__(parent, bg=wiz.BG)
        self._wiz = wiz
        self._install_dir = None
        self._python_exe  = None
        tk.Label(self, text="Installation Complete",
                 bg=wiz.BG, fg=wiz.FG, font=("Segoe UI", 13, "bold")).pack(
            anchor="w", pady=(20, 6))
        self._msg = tk.Label(self, text="", bg=wiz.BG, fg=wiz.FG,
                             font=("Segoe UI", 10), justify="left")
        self._msg.pack(anchor="w")
        self._launch_btn = tk.Button(
            self, text="Launch SmartGrid", bg="#0078D4", fg="#ffffff",
            font=("Segoe UI", 10, "bold"), relief="flat", padx=12, pady=4,
            command=self._launch)
        self._launch_btn.pack(anchor="w", pady=(20, 0))
        tk.Button(self, text="Close", bg="#ffffff", fg=wiz.FG, relief="flat",
                  bd=1, highlightbackground="#d2d0ce",
                  command=wiz.destroy).pack(anchor="w", pady=(8, 0))

    def set_install_dir(self, install_dir, python_exe):
        self._install_dir = install_dir
        self._python_exe  = python_exe
        if python_exe:
            self._msg.config(text=(
                f"SmartGrid has been installed to:\n  {install_dir}\n\n"
                "A shortcut has been placed on your Desktop.\n"
                "Click Launch SmartGrid to start it now."
            ))
        else:
            self._msg.config(
                text="Installation encountered errors.\nSee the previous page for details.",
                fg="#c50f1f")
            self._launch_btn.config(state="disabled")

    def _launch(self):
        if not self._install_dir or not self._python_exe:
            return
        pythonw = self._python_exe.replace("python.exe", "pythonw.exe")
        if not os.path.exists(pythonw):
            pythonw = self._python_exe
        main_py = os.path.join(self._install_dir, "main.py")
        subprocess.Popen([pythonw, main_py], cwd=self._install_dir)
        self._wiz.destroy()


if __name__ == "__main__":
    Wizard().mainloop()
'''

# ---------------------------------------------------------------------------
# Build steps
# ---------------------------------------------------------------------------

def main():
    BUILD.mkdir(exist_ok=True)

    # 1 — zip project
    zip_path = BUILD / 'smartgrid_src.zip'
    print("Zipping project…")
    with zipfile.ZipFile(zip_path, 'w', zipfile.ZIP_DEFLATED) as zf:
        for p in ROOT.rglob('*'):
            if any(s in p.parts for s in SKIP):
                continue
            if p.suffix in ('.pyc', '.spec'):
                continue
            if p == zip_path:
                continue
            if p.name == 'build_installer.py':
                continue
            if p.is_file():
                zf.write(p, p.relative_to(ROOT))
    print(f"  → {zip_path} ({zip_path.stat().st_size // 1024} KB)")

    # 2 — embed zip in installer.py
    b64 = base64.b64encode(zip_path.read_bytes()).decode()
    installer_src = INSTALLER_TEMPLATE.replace('__EMBEDDED_ZIP__', b64)
    installer_py = BUILD / 'installer.py'
    installer_py.write_text(installer_src, encoding='utf-8')
    print(f"  → {installer_py} ({installer_py.stat().st_size // 1024} KB)")

    # 3 — ensure PyInstaller
    try:
        import PyInstaller  # noqa: F401
    except ImportError:
        print("Installing PyInstaller…")
        subprocess.check_call([sys.executable, '-m', 'pip', 'install', 'pyinstaller'])

    # 4 — run PyInstaller
    desktop = pathlib.Path.home() / 'Desktop'
    print("Running PyInstaller…")
    import PyInstaller.__main__  # noqa: E402
    PyInstaller.__main__.run([
        '--onefile',
        '--windowed',
        '--name=SmartGrid_Setup',
        f'--distpath={desktop}',
        f'--workpath={BUILD / "pyinstaller_work"}',
        f'--specpath={BUILD}',
        str(installer_py),
    ])

    out = desktop / 'SmartGrid_Setup.exe'
    if out.exists():
        print(f'\nDone → {out}')
    else:
        print('\nWarning: exe not found at expected path — check PyInstaller output above.')


if __name__ == '__main__':
    main()
