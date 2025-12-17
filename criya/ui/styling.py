# Stylesheets and Colors

# Professional Light Theme
BG_MAIN  = "#F3F4F6"     # Cool Gray 100
BG_CARD  = "#FFFFFF"     # White
PRIMARY  = "#2563EB"     # Royal Blue (Tailwind Blue 600)
PRIM_HOVER = "#1D4ED8"   # Darker Blue
TEXT     = "#111827"     # Gray 900
TEXT_SUB = "#4B5563"     # Gray 600
BORDER   = "#D1D5DB"     # Gray 300

# Status Colors
OK_GREEN = "#10B981"     # Emerald 500
WARN_AMB = "#F59E0B"     # Amber 500
ERR_RED  = "#EF4444"     # Red 500
SECONDARY= "#64748B"     # Slate 500

STYLE_SHEET = f"""
QMainWindow {{
    background-color: {BG_MAIN};
}}
QWidget {{
    font-family: "Segoe UI", "Inter", sans-serif;
    font-size: 13px;
    color: {TEXT};
}}

/* Cards (GroupBoxes) */
QGroupBox {{
    background-color: {BG_CARD};
    border: 1px solid {BORDER};
    border-radius: 6px;
    margin-top: 24px; /* Space for title */
    padding-top: 10px;
    font-weight: bold;
}}
QGroupBox::title {{
    subcontrol-origin: margin;
    subcontrol-position: top left;
    padding: 0 5px;
    left: 10px;
    color: {PRIMARY};
    font-size: 12px;
    text-transform: uppercase;
    background-color: transparent; 
}}

/* Scroll Areas */
QScrollArea {{ border: none; background: transparent; }}
QScrollArea > QWidget > QWidget {{ background: transparent; }}

/* Connect Input Fields */
QLineEdit, QComboBox, QSpinBox {{
    border: 1px solid {BORDER};
    border-radius: 4px;
    padding: 6px 10px;
    background: #FFFFFF;
    color: {TEXT};
    selection-background-color: {PRIMARY};
}}
QLineEdit:focus, QComboBox:focus {{
    border: 1px solid {PRIMARY};
}}
QComboBox::drop-down {{
    border: none; margin-right: 5px; width: 20px;
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
    background-color: {BG_CARD};
    border: 1px solid {BORDER};
    color: {TEXT};
    border-radius: 4px;
    padding: 8px 16px;
    font-weight: 600;
}}
QPushButton:hover {{
    background-color: #F9FAFB;
    border-color: {PRIMARY};
    color: {PRIMARY};
}}
QPushButton:pressed {{
    background-color: #EFF6FF;
}}

/* Primary Actions (Connect, Save) - Use explicit property or default to secondary style */
QPushButton[text="Connect"], QPushButton[text="Upload Firmware"], QPushButton[text="Save Current"], QPushButton[text="Move to Point"] {{
    background-color: {PRIMARY};
    color: white;
    border: 1px solid {PRIMARY};
}}
QPushButton[text="Connect"]:hover, QPushButton[text="Upload Firmware"]:hover {{
    background-color: {PRIM_HOVER};
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
}}

/* Sliders */
QSlider::groove:horizontal {{
    height: 4px;
    background: {BORDER};
    border-radius: 2px;
}}
QSlider::sub-page:horizontal {{
    background: {PRIMARY};
    border-radius: 2px;
}}
QSlider::handle:horizontal {{
    background: #FFFFFF;
    border: 1px solid {BORDER};
    width: 16px;
    height: 16px;
    margin: -6px 0; /* Center on groove */
    border-radius: 8px;
}}
QSlider::handle:horizontal:hover {{
    border-color: {PRIMARY};
}}

/* List Widget */
QListWidget {{
    background: {BG_CARD};
    border: 1px solid {BORDER};
    border-radius: 4px;
}}
QListWidget::item {{
    padding: 6px;
    border-bottom: 1px solid #F3F4F6;
}}
QListWidget::item:selected {{
    background-color: {PRIMARY};
    color: white;
}}

/* Table Widget */
QTableWidget {{
    background-color: {BG_CARD};
    gridline-color: {BG_MAIN};
    border: 1px solid {BORDER};
}}
QHeaderView::section {{
    background-color: {BG_MAIN};
    padding: 6px;
    border: none;
    font-weight: bold;
    color: {TEXT_SUB};
}}

"""
