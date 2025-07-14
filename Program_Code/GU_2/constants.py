from kivy.utils import get_color_from_hex
from kivy.core.window import Window

COLOR_BACKGROUND_DARK = get_color_from_hex('#0d2137')
COLOR_BACKGROUND_LIGHT = get_color_from_hex('#2E77AE')
COLOR_TEXT_LIGHT = get_color_from_hex('#E0EAF5')
COLOR_ACCENT_ORANGE = get_color_from_hex('#FF8E2B')
COLOR_ACCENT_GREEN = get_color_from_hex('#2ECC71')
COLOR_ACCENT_RED = get_color_from_hex('#E74C3C')

# Set window size and background
Window.size = (1600, 1000)
Window.clearcolor = COLOR_BACKGROUND_DARK