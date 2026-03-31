"""
================================================================================
 COBRA VTOL — Camera Discovery Test  (Windows / DirectShow)
 Scans device indices 0..9 and reports which ones deliver frames.
 Run this to verify OBS Virtual Camera is visible before launching the GCS.
================================================================================
"""

import cv2
import sys


def test_cameras(max_index: int = 10) -> None:
    print("=" * 60)
    print("  DirectShow Camera Scanner  (Windows)")
    print("=" * 60)

    found = 0

    for idx in range(max_index):
        print(f"\n[{idx}]  Probing index {idx} with CAP_DSHOW ... ", end="")
        cap = cv2.VideoCapture(idx, cv2.CAP_DSHOW)

        if not cap.isOpened():
            print("NOT FOUND")
            continue

        ret, frame = cap.read()
        if ret:
            h, w = frame.shape[:2]
            print(f"OK  →  {w}×{h}")
            found += 1
        else:
            print("OPENED but no frame delivered")

        cap.release()

    print("\n" + "=" * 60)
    if found:
        print(f"  {found} camera(s) detected.  OBS Virtual Camera should be one of them.")
    else:
        print("  No cameras detected!")
        print("  → Make sure OBS is running and Virtual Camera is started.")
    print("=" * 60)


if __name__ == "__main__":
    test_cameras()
