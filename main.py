"""Entry point: creates QApplication, shows MainWindow."""
from __future__ import annotations
import sys


_APP_STYLE = (
    # Dialogs, message boxes, input dialogs — light Office-style
    'QWidget{font-family:"Segoe UI",Arial;font-size:12px;color:#252525;}'
    'QDialog,QMessageBox{background:#ffffff;}'
    'QLabel{color:#252525;background:transparent;}'
    'QRadioButton,QCheckBox{color:#252525;background:transparent;spacing:6px;}'
    'QRadioButton::indicator,QCheckBox::indicator{width:14px;height:14px;'
    '  border:1px solid #8a8886;border-radius:7px;background:#fff;}'
    'QRadioButton::indicator:checked{background:#0078d4;border-color:#0078d4;}'
    'QCheckBox::indicator{border-radius:3px;}'
    'QCheckBox::indicator:checked{background:#0078d4;border-color:#0078d4;}'
    'QPushButton{background:#fff;color:#252525;border:1px solid #d2d0ce;'
    '  border-radius:2px;padding:4px 14px;min-width:64px;}'
    'QPushButton:hover{background:#edebe9;}'
    'QPushButton:pressed{background:#d2d0ce;}'
    'QPushButton:default{background:#0078d4;color:#fff;border-color:#0078d4;}'
    'QPushButton:default:hover{background:#106ebe;}'
    'QLineEdit,QTextEdit,QPlainTextEdit{background:#fff;color:#252525;'
    '  border:1px solid #d2d0ce;border-radius:2px;padding:2px 4px;}'
    'QLineEdit:focus,QTextEdit:focus{border-color:#0078d4;}'
    'QComboBox{background:#fff;color:#252525;border:1px solid #d2d0ce;'
    '  border-radius:2px;padding:3px 6px;min-width:120px;}'
    'QComboBox:hover{border-color:#8a8886;}'
    'QComboBox::drop-down{border:none;width:20px;}'
    'QComboBox QAbstractItemView{background:#fff;color:#252525;'
    '  selection-background-color:#0078d4;selection-color:#fff;'
    '  border:1px solid #d2d0ce;}'
    'QDialogButtonBox QPushButton{min-width:72px;padding:5px 16px;}'
    'QScrollBar:vertical{background:#f3f2f1;width:10px;border:none;}'
    'QScrollBar::handle:vertical{background:#c8c6c4;border-radius:5px;min-height:20px;}'
    'QScrollBar::add-line,QScrollBar::sub-line{height:0;}'
)


def main() -> None:
    # QApplication must be created before any PyVista/VTK initialisation
    from PySide6.QtWidgets import QApplication
    app = QApplication(sys.argv)
    app.setApplicationName("SmartGrid")
    app.setApplicationVersion("1.3")
    app.setStyleSheet(_APP_STYLE)

    # Deferred import so Qt is ready before VTK registers its OpenGL context
    from app.main_window import MainWindow
    window = MainWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
