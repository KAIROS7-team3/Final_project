string label            # "Left" or "Right"
float32 score           # detection confidence [0, 1]
float32[] landmarks_canon   # 63 floats: 21 landmarks × (u, v, z_norm) — pixel coords
float32[] landmarks_world   # 63 floats: 21 landmarks × (x, y, z) — MediaPipe world [m]
