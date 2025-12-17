import sys
import logging
from PySide6 import QtWidgets, QtGui, QtCore

# Disable qt_material usage to ensure clean, simple professional look
# try:
#     from qt_material import apply_stylesheet
#     HAVE_QT_MATERIAL = True
# except ImportError:
#     HAVE_QT_MATERIAL = False

from criya.ui.main_window import MainWindow
# Import all constants needed for the app
from criya.ui.styling import STYLE_SHEET

def main():
    # Setup Logging
    logging.basicConfig(
        filename="botlog.txt",
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(message)s",
        filemode='a' 
    )
    logging.info("=== Application Started (Refactored) ===")

    app = QtWidgets.QApplication(sys.argv)
    app.setApplicationName("Criya Robot")
    
    # Use Fusion as base for cross-platform consistency
    app.setStyle("Fusion")
    
    # Apply our Custom Professional Stylesheet
    app.setStyleSheet(STYLE_SHEET)
    
    win = MainWindow()
    win.show()
    
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
