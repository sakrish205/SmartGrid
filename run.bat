@echo off
cd /d "%~dp0"
set SMARTGRID_MESH_FLOW_UI=1
start "" pythonw main.py
