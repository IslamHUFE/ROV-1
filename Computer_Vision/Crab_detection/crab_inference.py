"""
Crab Detection Inference Module
================================
Uses a YOLO model to detect crabs and returns bounding boxes + counts.

Usage:
    from crab_inference import CrabDetector

    detector = CrabDetector(model_path="path/to/best.pt")
    results  = detector.predict("image.jpg")
    print(results)
"""

from ultralytics import YOLO
from pathlib import Path
import cv2
import numpy as np


class CrabDetector:
    """
    Wrapper around a YOLO model trained to detect crab species.

    Parameters
    ----------
    model_path : str | Path
        Path to the YOLO weights file (e.g. best.pt).
    conf_threshold : float
        Minimum confidence score to keep a detection (default: 0.25).
    device : str
        Inference device: "cpu", "cuda", "mps", etc. (default: "cpu").
    """

    def __init__(
        self,
        model_path: str | Path,
        conf_threshold: float = 0.25,
        device: str = "cpu",
    ):
        self.model = YOLO(str(model_path))
        self.conf_threshold = conf_threshold
        self.device = device

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def predict(self, source) -> dict:
        """
        Run inference on a single image.

        Parameters
        ----------
        source : str | Path | np.ndarray
            Image path, URL, or a BGR/RGB numpy array.

        Returns
        -------
        dict with keys:
            total_count  : int          – total crabs detected
            class_counts : dict[str,int]– per-species counts
            detections   : list[dict]   – one entry per detected crab
                ├─ class_id   : int
                ├─ class_name : str
                ├─ confidence : float
                └─ bbox       : dict { x1, y1, x2, y2, width, height }
        """
        raw = self.model.predict(
            source,
            conf=self.conf_threshold,
            device=self.device,
            verbose=False,
        )

        return self._parse_results(raw[0])

    def predict_batch(self, sources: list) -> list[dict]:
        """
        Run inference on a list of images.

        Returns a list of result dicts (same schema as predict()).
        """
        raw = self.model.predict(
            sources,
            conf=self.conf_threshold,
            device=self.device,
            verbose=False,
        )
        return [self._parse_results(r) for r in raw]

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _parse_results(self, result) -> dict:
        names = result.names           # {class_id: "class_name", ...}
        boxes = result.boxes           # ultralytics Boxes object

        detections   = []
        class_counts = {}

        if boxes is not None and len(boxes):
            for box in boxes:
                cls_id   = int(box.cls.item())
                cls_name = names[cls_id]
                conf     = round(float(box.conf.item()), 4)
                x1, y1, x2, y2 = [round(float(v), 2) for v in box.xyxy[0]]

                detections.append(
                    {
                        "class_id"  : cls_id,
                        "class_name": cls_name,
                        "confidence": conf,
                        "bbox"      : {
                            "x1"    : x1,
                            "y1"    : y1,
                            "x2"    : x2,
                            "y2"    : y2,
                            "width" : round(x2 - x1, 2),
                            "height": round(y2 - y1, 2),
                        },
                    }
                )

                class_counts[cls_name] = class_counts.get(cls_name, 0) + 1

        return {
            "total_count" : len(detections),
            "class_counts": class_counts,
            "detections"  : detections,
        }


# ----------------------------------------------------------------------
# Optional: draw bounding boxes for debugging / visualisation
# ----------------------------------------------------------------------

def draw_detections(image_path: str | Path, results: dict, output_path: str | Path = "output.jpg"):
    """
    Draws bounding boxes on the image and saves it.
    Useful for visual debugging – not required for production use.
    """
    img = cv2.imread(str(image_path))
    if img is None:
        raise FileNotFoundError(f"Image not found: {image_path}")

    for det in results["detections"]:
        b    = det["bbox"]
        x1, y1, x2, y2 = int(b["x1"]), int(b["y1"]), int(b["x2"]), int(b["y2"])
        label = f"{det['class_name']} {det['confidence']:.2f}"

        cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(
            img, label, (x1, y1 - 8),
            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2,
        )

    cv2.imwrite(str(output_path), img)
    print(f"Annotated image saved → {output_path}")


# ----------------------------------------------------------------------
# Quick CLI test:  python crab_inference.py --model best.pt --image test.jpg
# ----------------------------------------------------------------------

if __name__ == "__main__":
    import argparse, json

    parser = argparse.ArgumentParser(description="Crab YOLO Inference")
    parser.add_argument("--model",      required=True, help="Path to YOLO weights (.pt)")
    parser.add_argument("--image",      required=True, help="Path to input image")
    parser.add_argument("--conf",       type=float, default=0.25, help="Confidence threshold")
    parser.add_argument("--device",     default="cpu", help="cpu | cuda | mps")
    parser.add_argument("--save-image", action="store_true", help="Save annotated image")
    args = parser.parse_args()

    detector = CrabDetector(
        model_path=args.model,
        conf_threshold=args.conf,
        device=args.device,
    )

    results = detector.predict(args.image)

    print("\n===== Crab Detection Results =====")
    print(f"Total crabs detected : {results['total_count']}")
    print(f"Per-species counts   : {results['class_counts']}")
    print("\nDetections:")
    print(json.dumps(results["detections"], indent=2))

    if args.save_image:
        draw_detections(args.image, results, output_path="crab_output.jpg")