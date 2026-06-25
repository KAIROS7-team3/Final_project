"""DeepFilterNet 3 오디오 노이즈 제거 모듈.

deepfilternet 패키지가 torchaudio 구버전 API(torchaudio.backend.common)에
의존하므로, import 전에 sys.modules에 호환 shim을 주입한다.
site-packages를 수정하지 않으며, enhance() 함수 자체는 정상 동작한다.

사용:
    enhancer = DeepFilterEnhancer(device="cuda")
    clean_audio = enhancer.enhance(noisy_audio_16k)  # float32 numpy, 16kHz
"""

from __future__ import annotations

import logging
import sys
from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np

logger = logging.getLogger(__name__)


def _inject_torchaudio_shim() -> None:
    """torchaudio.backend.common.AudioMetaData shim을 sys.modules에 주입한다.

    deepfilternet 0.5.x는 torchaudio 1.x의 내부 API를 직접 참조한다.
    torchaudio 2.x에서 해당 경로가 제거됐으므로, import 전에 빈 dataclass를
    sys.modules에 등록해 ImportError를 방지한다.
    """
    if "torchaudio.backend.common" in sys.modules:
        return

    try:
        import torchaudio  # noqa: F401 — 이미 설치돼 있어야 함
    except ImportError:
        return

    import types

    @dataclass
    class _AudioMetaData:
        sample_rate: int = 0
        num_frames: int = 0
        num_channels: int = 0
        bits_per_sample: int = 0
        encoding: str = ""

    shim = types.ModuleType("torchaudio.backend.common")
    shim.AudioMetaData = _AudioMetaData  # type: ignore[attr-defined]

    # torchaudio.backend 네임스페이스도 없을 수 있으므로 함께 등록한다.
    if "torchaudio.backend" not in sys.modules:
        backend_mod = types.ModuleType("torchaudio.backend")
        sys.modules["torchaudio.backend"] = backend_mod

    sys.modules["torchaudio.backend.common"] = shim


class DeepFilterLoadError(RuntimeError):
    """DeepFilterNet 3 모델 로드 실패."""


class DeepFilterEnhancer:
    """DeepFilterNet 3으로 16kHz 오디오의 노이즈를 제거한다.

    DeepFilterNet 3은 48kHz 모델이므로 내부적으로 16↔48kHz 리샘플링을 수행한다.
    model.to(device)로 GPU에 올려 처리하며, 녹음 후 1회성 호출이므로
    GPU 상주 시간이 짧다.
    """

    def __init__(self, device: str = "cuda") -> None:
        self._device = device
        self._model = None
        self._df_state = None
        self._df_sr: int = 48_000
        self._enhance_fn = None
        self._load()

    def _load(self) -> None:
        _inject_torchaudio_shim()
        try:
            from df import enhance as _enhance, init_df
        except ImportError as exc:
            raise DeepFilterLoadError(
                "deepfilternet 미설치: pip install deepfilternet"
            ) from exc

        try:
            model, df_state, _ = init_df()
        except Exception as exc:
            raise DeepFilterLoadError(f"DeepFilterNet 초기화 실패: {exc}") from exc

        import torch
        self._model = model.to(self._device)
        self._df_state = df_state
        # DF.sr()는 모델 내부 샘플레이트(보통 48000)를 반환한다
        self._df_sr = int(df_state.sr())
        self._enhance_fn = _enhance
        logger.info(
            f"[noise_filter] DeepFilterNet 3 로드 완료 "
            f"(device={self._device}, model_sr={self._df_sr}Hz)"
        )

    def enhance(self, audio_16k: np.ndarray) -> np.ndarray:
        """16kHz float32 오디오를 받아 노이즈 제거 후 16kHz로 반환한다."""
        import torch
        import torchaudio.functional as taF

        audio_t = torch.from_numpy(audio_16k).float().unsqueeze(0)  # (1, N)

        # 16kHz → model_sr (48kHz)
        if self._df_sr != SAMPLE_RATE_HZ:
            audio_t = taF.resample(audio_t, SAMPLE_RATE_HZ, self._df_sr)

        # enhance()는 Tensor를 받는다 (numpy 불가)
        enhanced_t = self._enhance_fn(self._model, self._df_state, audio_t)
        if not isinstance(enhanced_t, torch.Tensor):
            enhanced_t = torch.from_numpy(np.asarray(enhanced_t)).float()
        if enhanced_t.dim() == 1:
            enhanced_t = enhanced_t.unsqueeze(0)

        # model_sr → 16kHz
        if self._df_sr != SAMPLE_RATE_HZ:
            enhanced_t = taF.resample(enhanced_t, self._df_sr, SAMPLE_RATE_HZ)

        return enhanced_t.squeeze(0).numpy().astype(np.float32)


SAMPLE_RATE_HZ = 16_000
