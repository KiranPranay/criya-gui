# Stylesheets and Colors

# Premium Light Theme
BG_MAIN  = "#F8F9FA"     # Very Light Gray (Almost White)
BG_CARD  = "#FFFFFF"     # Pure White
PRIMARY  = "#0066CC"     # Professional Blue
PRIM_HOVER = "#0052A3"   # Darker Blue
TEXT     = "#1F2937"     # Dark Slate
TEXT_SUB = "#6B7280"     # Medium Gray
BORDER   = "#E5E7EB"     # Light Border

# Status Colors
OK_GREEN = "#10B981"     # Emerald
WARN_AMB = "#D97706"     # Amber (Readable on white)
ERR_RED  = "#DC2626"     # Red
SECONDARY= "#64748B"     # Slate

STYLE_SHEET = f"""
QMainWindow {{
    background-color: {BG_MAIN};
}}
QWidget {{
    font-family: "Segoe UI", "Roboto", "Helvetica Neue", sans-serif;
    font-size: 13px;
    color: {TEXT};
}}

/* Cards (GroupBoxes) */
QGroupBox {{
    background-color: {BG_CARD};
    border: 1px solid {BORDER};
    border-radius: 8px;
    margin-top: 1.5em; /* Space for title */
    padding-top: 12px;
    padding-bottom: 12px;
    padding-left: 12px;
    padding-right: 12px; 
    font-weight: 600;
}}
QGroupBox::title {{
    subcontrol-origin: margin;
    subcontrol-position: top left;
    padding: 0 8px;
    left: 12px;
    top: 2px;
    color: {PRIMARY};
    font-weight: 700;
    font-size: 13px;
    text-transform: uppercase;
    background-color: transparent; 
}}

/* Scroll Areas */
QScrollArea {{ border: none; background: transparent; }}
QScrollArea > QWidget > QWidget {{ background: transparent; }}

/* Connect Input Fields */
QLineEdit, QComboBox, QSpinBox {{
    border: 1px solid {BORDER};
    border-radius: 6px;
    padding: 6px 12px;
    background: #FFFFFF;
    color: {TEXT};
    min-height: 20px;
}}
QLineEdit:focus, QComboBox:focus {{
    border: 1px solid {PRIMARY};
    background: #FFFFFF;
}}
QComboBox::drop-down {{
    border: none; margin-right: 8px; width: 20px;
}}
QComboBox::down-arrow {{
    image: none;
    border-left: 5px solid transparent;
    border-right: 5px solid transparent;
    border-top: 5px solid {TEXT_SUB};
    margin: 0;
}}

/* Buttons */
QPushButton {{
    background-color: #FFFFFF;
    border: 1px solid {BORDER};
    color: {TEXT};
    border-radius: 6px;
    padding: 8px 16px;
    font-weight: 600;
    font-size: 13px;
    min-width: 80px;
}}
QPushButton:hover {{
    background-color: #F3F4F6;
    border-color: {PRIMARY};
    color: {PRIMARY};
}}
QPushButton:pressed {{
    background-color: #E5E7EB;
}}

/* Primary Actions */
QPushButton[text="Connect"], QPushButton[text="Upload Firmware"], QPushButton[text="Save Current"], QPushButton[text="Move to Point"] {{
    background-color: {PRIMARY};
    color: white;
    border: 1px solid {PRIMARY};
}}
QPushButton[text="Connect"]:hover, QPushButton[text="Upload Firmware"]:hover {{
    background-color: {PRIM_HOVER};
    border-color: {PRIM_HOVER};
}}

/* Danger Actions */
QPushButton[danger="true"] {{
    background-color: #FEF2F2;
    color: {ERR_RED};
    border-color: #FECACA;
}}
QPushButton[danger="true"]:hover {{
    background-color: {ERR_RED};
    color: white;
    border-color: {ERR_RED};
}}

/* Warning Actions */
QPushButton[warn="true"] {{
    background-color: #FFFBEB;
    color: {WARN_AMB};
    border-color: #FDE68A;
}}
QPushButton[warn="true"]:hover {{
    background-color: {WARN_AMB};
    color: white;
    border-color: {WARN_AMB};
}}

/* Sliders */
QSlider::groove:horizontal {{
    height: 6px;
    background: {BORDER};
    border-radius: 3px;
}}
QSlider::sub-page:horizontal {{
    background: {PRIMARY};
    border-radius: 3px;
}}
QSlider::handle:horizontal {{
    background: #FFFFFF;
    border: 1px solid {BORDER};
    width: 18px;
    height: 18px;
    margin: -7px 0; /* Center on groove */
    border-radius: 9px;
}}
QSlider::handle:horizontal:hover {{
    border-color: {PRIMARY};
}}

/* List & Tables */
QListWidget, QTableWidget {{
    background: {BG_CARD};
    border: 1px solid {BORDER};
    border-radius: 6px;
    alternate-background-color: #F9FAFB;
}}
QListWidget::item {{
    padding: 8px;
    border-bottom: 1px solid #F3F4F6;
}}
QListWidget::item:selected, QTableWidget::item:selected {{
    background-color: {PRIMARY};
    color: white;
    border: none;
}}
QHeaderView::section {{
    background-color: {BG_MAIN};
    padding: 8px;
    border: none;
    border-bottom: 1px solid {BORDER};
    font-weight: bold;
    color: {TEXT_SUB};
    text-transform: uppercase;
    font-size: 11px;
}}

/* Splitter */
QSplitter::handle {{
    background: {BORDER};
    width: 1px;
}}

/* Tabs */
QTabWidget::pane {{
    border: 1px solid {BORDER};
    background: {BG_CARD};
    border-radius: 6px;
}}
QTabBar::tab {{
    background: {BG_MAIN};
    border: 1px solid {BORDER};
    padding: 8px 20px;
    margin-right: 4px;
    border-top-left-radius: 6px;
    border-top-right-radius: 6px;
    color: {TEXT_SUB};
}}
QTabBar::tab:selected {{
    background: {BG_CARD};
    border-bottom-color: {BG_CARD}; /* Blend with pane */
    color: {PRIMARY};
    font-weight: bold;
}}

"""
