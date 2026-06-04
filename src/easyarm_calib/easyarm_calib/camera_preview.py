"""Preview images from a Daheng Galaxy camera."""

from pathlib import Path
import select
import sys
import termios
import time
import tty

import cv2


CAMERA_INDEX = 1
EXPOSURE_TIME_US = 5000.0
GAIN_DB = 10.0
FRAME_RATE_HZ = 30.0
ACQUIRE_TIMEOUT_MS = 1000
WINDOW_NAME = "Daheng Camera Preview"
MAX_PREVIEW_SIZE = 640
CAPTURE_ROOT = Path("data/camera_capture")


class TerminalKeyReader:
    """Read single keys from the terminal without blocking preview refresh."""

    def __init__(self):
        self._enabled = sys.stdin.isatty()
        self._old_attrs = None
        if self._enabled:
            self._old_attrs = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())

    def read_key(self):
        if not self._enabled:
            return None
        readable, _, _ = select.select([sys.stdin], [], [], 0)
        if not readable:
            return None
        return sys.stdin.read(1)

    def close(self):
        if self._enabled and self._old_attrs is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old_attrs)


def _find_gxipy_api_path():
    candidates = []
    for parent in Path(__file__).resolve().parents:
        candidates.append(parent / "ref")
    candidates.append(Path.cwd() / "ref")

    for ref_dir in candidates:
        if not ref_dir.is_dir():
            continue
        for api_dir in ref_dir.glob("Galaxy_Linux_Python*/Galaxy_Linux_Python*/api"):
            if (api_dir / "gxipy").is_dir():
                return api_dir
    return None


def _import_gxipy():
    api_path = _find_gxipy_api_path()
    if api_path is not None:
        sys.path.insert(0, str(api_path))

    try:
        import gxipy as gx  # pylint: disable=import-outside-toplevel
    except ImportError as exc:
        raise RuntimeError(
            "Cannot import gxipy. Install the Galaxy Python API or keep "
            "ref/Galaxy_Linux_Python_*/Galaxy_Linux_Python_*/api in this workspace."
        ) from exc

    return gx


def _set_feature_if_writable(feature, value, name):
    if not feature.is_implemented() or not feature.is_writable():
        print(f"Skip {name}: not implemented or not writable")
        return

    feature.set(value)
    print(f"Set {name}: {value}")


def _configure_camera(gx, cam):
    cam.TriggerMode.set(gx.GxSwitchEntry.OFF)
    _set_feature_if_writable(cam.ExposureTime, EXPOSURE_TIME_US, "ExposureTime(us)")
    _set_feature_if_writable(cam.Gain, GAIN_DB, "Gain(dB)")

    if cam.AcquisitionFrameRateMode.is_implemented() and cam.AcquisitionFrameRateMode.is_writable():
        cam.AcquisitionFrameRateMode.set(gx.GxSwitchEntry.ON)
    _set_feature_if_writable(cam.AcquisitionFrameRate, FRAME_RATE_HZ, "AcquisitionFrameRate(Hz)")


def _color_improvement_params(gx, cam):
    gamma_lut = None
    contrast_lut = None
    color_correction_param = 0

    if cam.GammaParam.is_readable():
        gamma_lut = gx.Utility.get_gamma_lut(cam.GammaParam.get())
    if cam.ContrastParam.is_readable():
        contrast_lut = gx.Utility.get_contrast_lut(cam.ContrastParam.get())
    if cam.ColorCorrectionParam.is_readable():
        color_correction_param = cam.ColorCorrectionParam.get()

    return color_correction_param, contrast_lut, gamma_lut


def _raw_image_to_bgr(raw_image, color_camera, improvement_params):
    if color_camera:
        rgb_image = raw_image.convert("RGB")
        if rgb_image is None:
            return None
        rgb_image.image_improvement(*improvement_params)
        frame = rgb_image.get_numpy_array()
        if frame is None:
            return None
        return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    frame = raw_image.get_numpy_array()
    if frame is None:
        return None
    return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)


def _resize_for_display(frame):
    height, width = frame.shape[:2]
    longest_side = max(width, height)
    if longest_side <= MAX_PREVIEW_SIZE:
        return frame

    scale = MAX_PREVIEW_SIZE / longest_side
    return cv2.resize(
        frame,
        None,
        fx=scale,
        fy=scale,
        interpolation=cv2.INTER_AREA,
    )


def _create_capture_dir():
    now = time.localtime()
    capture_dir = CAPTURE_ROOT / time.strftime("%Y%m%d", now) / time.strftime("%H%M%S", now)
    capture_dir.mkdir(parents=True, exist_ok=True)
    return capture_dir


def _read_key(terminal_reader):
    terminal_key = terminal_reader.read_key()
    if terminal_key is not None:
        return terminal_key

    cv_key = cv2.waitKeyEx(10)
    if cv_key < 0:
        return None
    if cv_key == 27:
        return "\x1b"
    if 0 <= cv_key <= 255:
        return chr(cv_key)
    return None


def _save_capture(frame, capture_dir, capture_index):
    if capture_dir is None:
        capture_dir = _create_capture_dir()
        print(f"Capture directory: {capture_dir}")

    image_path = capture_dir / f"IMG{capture_index:04d}.png"
    if cv2.imwrite(str(image_path), frame):
        print(f"Saved {image_path}")
        return capture_dir, capture_index + 1

    print(f"Failed to save {image_path}")
    return capture_dir, capture_index


def main():
    """Open the first Daheng Galaxy camera and show a live preview."""
    gx = _import_gxipy()

    device_manager = gx.DeviceManager()
    dev_num, dev_info_list = device_manager.update_device_list()
    if dev_num == 0:
        print("No Daheng Galaxy camera found")
        return

    print(f"Found {dev_num} camera(s)")
    for index, info in enumerate(dev_info_list, start=1):
        print(f"[{index}] {info}")

    cam = device_manager.open_device_by_index(CAMERA_INDEX)
    stream_on = False
    terminal_reader = TerminalKeyReader()

    try:
        color_camera = cam.PixelColorFilter.is_implemented()
        print("Camera type:", "color" if color_camera else "mono")

        _configure_camera(gx, cam)
        improvement_params = _color_improvement_params(gx, cam) if color_camera else None

        cam.stream_on()
        stream_on = True
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        print("Preview started. Press q/Q or Esc to exit.")
        print("Press c/C to save the current image.")

        last_report_time = time.monotonic()
        frames_since_report = 0
        capture_dir = None
        capture_index = 1

        while True:
            raw_image = cam.data_stream[0].get_image(timeout=ACQUIRE_TIMEOUT_MS)
            if raw_image is None:
                print("Getting image failed")
                continue

            frame = _raw_image_to_bgr(raw_image, color_camera, improvement_params)
            if frame is None:
                print("Converting image failed")
                continue

            frames_since_report += 1
            now = time.monotonic()
            if now - last_report_time >= 1.0:
                fps = frames_since_report / (now - last_report_time)
                print(
                    "Frame ID: %d  Size: %dx%d  FPS: %.1f"
                    % (
                        raw_image.get_frame_id(),
                        raw_image.get_width(),
                        raw_image.get_height(),
                        fps,
                    )
                )
                last_report_time = now
                frames_since_report = 0

            cv2.imshow(WINDOW_NAME, _resize_for_display(frame))
            key = _read_key(terminal_reader)
            if key in ("\x1b", "q", "Q"):
                break
            if key in ("c", "C"):
                capture_dir, capture_index = _save_capture(frame, capture_dir, capture_index)

    except KeyboardInterrupt:
        print("Preview interrupted")
    finally:
        terminal_reader.close()
        if stream_on:
            cam.stream_off()
        cam.close_device()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
