import math


class CandidateTracker:
    # Detector geometry changes every frame and must not create a new logical
    # target.  Only semantic identity fields participate in de-duplication.
    _GEOMETRY_KEYS = {
        "points", "bbox", "center_u", "center_v", "u", "v", "cx", "cy",
        "depth", "area", "side_px", "score", "stable_count", "reason",
        "point_camera", "real_width", "real_height", "valid_depth_ratio",
        "depth_std", "pixel_area", "fill_ratio", "method", "preprocess",
        "validated", "confirmable",
        "detector_stable", "detector_confirmable",
    }

    def __init__(
        self,
        distance_m=0.30,
        confirm_hits=1,
        confirm_hits_by_type=None,
        confirmation_max_gap_s_by_type=None,
        confirmation_duration_s_by_type=None,
        color_image_distance_px=80.0,
        color_image_max_gap_s=1.0,
    ):
        self.distance_m = float(distance_m)
        self.color_image_distance_px = max(0.0, float(color_image_distance_px))
        self.color_image_max_gap_s = max(0.0, float(color_image_max_gap_s))
        self.confirm_hits = max(1, int(confirm_hits))
        self.confirm_hits_by_type = {
            str(key): max(1, int(value))
            for key, value in (confirm_hits_by_type or {}).items()
        }
        self.confirmation_max_gap_s_by_type = {
            str(key): max(0.0, float(value))
            for key, value in (confirmation_max_gap_s_by_type or {}).items()
        }
        self.confirmation_duration_s_by_type = {
            str(key): max(0.0, float(value))
            for key, value in (confirmation_duration_s_by_type or {}).items()
        }
        self._items = []
        self._next = {}

    @staticmethod
    def _image_center(result):
        if "center_u" in result and "center_v" in result:
            return float(result["center_u"]), float(result["center_v"])
        bbox = result.get("bbox")
        if isinstance(bbox, (list, tuple)) and len(bbox) == 4:
            x, y, width, height = (float(value) for value in bbox)
            return x + width * 0.5, y + height * 0.5
        return None

    @staticmethod
    def _result_key(result):
        return tuple(
            sorted(
                (str(k), str(v))
                for k, v in result.items()
                if k not in CandidateTracker._GEOMETRY_KEYS
            )
        )

    def update(
        self,
        target_type,
        result,
        position,
        allow_confirmation=True,
        timestamp=None,
    ):
        key = self._result_key(result)
        current_stamp = None if timestamp is None else float(timestamp)
        image_center = self._image_center(result)
        qr_content = ""
        if target_type == "qr_code":
            qr_content = str(result.get("content", "") or "").strip()
        best = None
        best_match = None
        for item in self._items:
            if item["target_type"] != target_type:
                continue
            same_decoded_qr = bool(
                qr_content
                and item.get("qr_content", "") == qr_content
            )
            if item["result_key"] != key and not same_decoded_qr:
                continue
            distance = math.sqrt(sum((float(position[a]) - item["position"][a]) ** 2 for a in ("x", "y", "z")))
            image_distance = None
            if image_center is not None and item.get("image_center") is not None:
                image_distance = math.hypot(
                    image_center[0] - item["image_center"][0],
                    image_center[1] - item["image_center"][1],
                )
            last_seen_stamp = item.get("last_seen_stamp")
            image_gap = (
                None
                if current_stamp is None or last_seen_stamp is None
                else current_stamp - last_seen_stamp
            )
            same_color_image_track = (
                target_type == "color_tag"
                and image_distance is not None
                and image_gap is not None
                and 0.0 <= image_gap <= self.color_image_max_gap_s
                and image_distance <= self.color_image_distance_px
            )
            # A non-empty decoded QR payload is a stronger identity than its
            # drifting world coordinate.  Once registered, the same payload
            # must not be uploaded again after a FAST-LIO/TF position jump.
            if same_decoded_qr:
                match = (0, distance)
            elif distance <= self.distance_m:
                match = (1, distance)
            elif same_color_image_track:
                match = (2, image_distance)
            else:
                continue
            if best_match is None or match < best_match:
                best, best_match = item, match
        if best is None:
            number = self._next.get(target_type, 0) + 1
            self._next[target_type] = number
            best = {
                "target_type": target_type,
                "result_key": key,
                "qr_content": qr_content,
                "position": dict(position),
                "image_center": image_center,
                "last_seen_stamp": current_stamp,
                "hits": 0,
                "last_confirmation_stamp": None,
                "first_confirmation_stamp": None,
                "reported": False,
                "local_number": number,
            }
            self._items.append(best)
        else:
            best["image_center"] = image_center
            best["last_seen_stamp"] = current_stamp
        if not allow_confirmation:
            # Keep the candidate position available for RViz, but do not let
            # unvalidated observations accumulate toward a confirmation.
            if not best["reported"]:
                best["hits"] = 0
                best["last_confirmation_stamp"] = None
                best["first_confirmation_stamp"] = None
            for axis in ("x", "y", "z"):
                best["position"][axis] = float(position[axis])
            return best, False

        max_gap = self.confirmation_max_gap_s_by_type.get(target_type)
        last_stamp = best.get("last_confirmation_stamp")
        if (
            max_gap is not None
            and current_stamp is not None
            and last_stamp is not None
            and (current_stamp < last_stamp or current_stamp - last_stamp > max_gap)
        ):
            best["hits"] = 0
            best["first_confirmation_stamp"] = None

        best["hits"] += 1
        if current_stamp is not None:
            if best.get("first_confirmation_stamp") is None:
                best["first_confirmation_stamp"] = current_stamp
            best["last_confirmation_stamp"] = current_stamp
        alpha = 1.0 / best["hits"]
        for axis in ("x", "y", "z"):
            best["position"][axis] += alpha * (float(position[axis]) - best["position"][axis])
        required_hits = self.confirm_hits_by_type.get(
            target_type, self.confirm_hits
        )
        required_duration = self.confirmation_duration_s_by_type.get(
            target_type, 0.0
        )
        first_stamp = best.get("first_confirmation_stamp")
        observed_duration = (
            current_stamp - first_stamp
            if current_stamp is not None and first_stamp is not None
            else 0.0
        )
        newly_confirmed = (
            best["hits"] >= required_hits
            and observed_duration >= required_duration
            and not best["reported"]
        )
        if newly_confirmed:
            best["reported"] = True
        return best, newly_confirmed
