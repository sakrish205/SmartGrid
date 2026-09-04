@echo off
cd /d "%~dp0"
python main.py
if %ERRORLEVEL% neq 0 (
    echo.
    echo ERROR: App failed to start. Installing dependencies...
    pip install PySide6 pyvista pyvistaqt "trimesh[easy]" numpy scipy
    python main.py
)
