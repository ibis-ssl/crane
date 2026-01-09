# Copyright (c) 2026 ibis-ssl
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""PCM Audio Output using PyAudio (matching Google's official sample)."""

import logging
import queue
import threading
from typing import Optional

import pyaudio

logger = logging.getLogger(__name__)

# Audio format settings (from Google's official sample)
FORMAT = pyaudio.paInt16
CHANNELS = 1
RECEIVE_SAMPLE_RATE = 24000
CHUNK_SIZE = 1024


class PcmAudioOutput:
    """
    PCM Audio Output for Gemini Live API response.

    Uses PyAudio with persistent stream (matching Google's official sample).
    """

    def __init__(
        self,
        sample_rate: int = RECEIVE_SAMPLE_RATE,
        channels: int = CHANNELS,
        device: Optional[str] = None,
    ):
        self._sample_rate = sample_rate
        self._channels = channels
        self._device_index = None  # TODO: device name to index conversion

        self._audio_queue: queue.Queue = queue.Queue()
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._pyaudio: Optional[pyaudio.PyAudio] = None
        self._stream = None

    def start(self) -> None:
        """Start audio output thread."""
        if self._running:
            return

        try:
            self._pyaudio = pyaudio.PyAudio()

            # Open persistent output stream
            self._stream = self._pyaudio.open(
                format=FORMAT,
                channels=self._channels,
                rate=self._sample_rate,
                output=True,
                frames_per_buffer=CHUNK_SIZE,
            )

            self._running = True
            self._thread = threading.Thread(target=self._playback_loop, daemon=True)
            self._thread.start()

            logger.info(
                f"Audio output started: {self._sample_rate}Hz, {self._channels}ch (PyAudio)"
            )
        except Exception as e:
            logger.error(f"Failed to start audio output: {e}")
            logger.warning("Audio output will be disabled.")
            if self._stream:
                self._stream.close()
                self._stream = None
            if self._pyaudio:
                self._pyaudio.terminate()
                self._pyaudio = None
            self._running = False

    def stop(self) -> None:
        """Stop audio output thread."""
        if not self._running:
            return

        self._running = False

        # Clear queue to unblock the thread
        while not self._audio_queue.empty():
            try:
                self._audio_queue.get_nowait()
            except queue.Empty:
                break

        # Signal thread to exit
        self._audio_queue.put(None)

        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None

        if self._stream:
            self._stream.stop_stream()
            self._stream.close()
            self._stream = None

        if self._pyaudio:
            self._pyaudio.terminate()
            self._pyaudio = None

        logger.info("Audio output stopped")

    def play(self, pcm_data: bytes) -> None:
        """
        Queue PCM audio data for playback.

        Args:
            pcm_data: Raw PCM audio bytes (16-bit signed, little-endian)
        """
        if not self._running or not self._stream:
            return

        self._audio_queue.put(pcm_data)

    def _playback_loop(self) -> None:
        """Background thread for audio playback."""
        while self._running:
            try:
                # Wait for audio data
                pcm_data = self._audio_queue.get(timeout=0.5)

                if pcm_data is None:
                    break

                # Write to persistent stream (like Google's official sample)
                if self._stream:
                    self._stream.write(pcm_data)

            except queue.Empty:
                continue
            except Exception as e:
                logger.error(f"Playback error: {e}")

    def clear_buffer(self) -> None:
        """Clear audio buffer (for barge-in support)."""
        while not self._audio_queue.empty():
            try:
                self._audio_queue.get_nowait()
            except queue.Empty:
                break
