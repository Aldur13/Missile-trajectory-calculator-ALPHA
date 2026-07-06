"""Interactive pixel-picking tool for marking the missile in a photo."""
from __future__ import annotations

from typing import Optional


def pick_pixel(image_path: str, window_name: Optional[str] = None) -> tuple:
    """Open `image_path`, let the user left-click the missile, Enter to confirm.

    Click again to move the mark before confirming. Escape cancels and raises
    RuntimeError.
    """
    import cv2

    window_name = window_name or image_path
    image = cv2.imread(image_path)
    if image is None:
        raise FileNotFoundError(image_path)

    clicked = {"point": None}

    def on_mouse(event, x, y, flags, userdata):
        if event == cv2.EVENT_LBUTTONDOWN:
            clicked["point"] = (x, y)

    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(window_name, on_mouse)

    while True:
        display = image.copy()
        if clicked["point"] is not None:
            cv2.drawMarker(display, clicked["point"], (0, 0, 255), cv2.MARKER_CROSS, 20, 2)
        cv2.imshow(window_name, display)
        key = cv2.waitKey(20) & 0xFF

        if key == 13 and clicked["point"] is not None:  # Enter
            break
        if key == 27:  # Escape
            cv2.destroyWindow(window_name)
            raise RuntimeError(f"Point selection cancelled for {image_path}")

    cv2.destroyWindow(window_name)
    return clicked["point"]
