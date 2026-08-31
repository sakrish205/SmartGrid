"""Entry point: creates QApplication, shows MainWindow."""
from __future__ import annotations
import sys


def main() -> None:
    # QApplication must be created before any PyVista/VTK initialisation
    from PySide6.QtWidgets import QApplication
    app = QApplication(sys.argv)
    app.setApplicationName("SurfaceCoat")
    app.setApplicationVersion("1.0.0")

    # Deferred import so Qt is ready before VTK registers its OpenGL context
    from app.main_window import MainWindow
    window = MainWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
