"""Tests for mmap-backed job queue — timestamp preservation."""

from inference_server.epoch_buffer import Epoch
from inference_server.mmap_queue import MmapJobQueue
from inference_server.zuna_worker import InferenceJob


def _make_job(headband_id: int, seq: int, timestamp_us: int) -> InferenceJob:
    channels = [[float(i) for i in range(8)] for _ in range(4)]
    epoch = Epoch(seq=seq, channels=channels, timestamp_us=timestamp_us)
    return InferenceJob(headband_id=headband_id, epoch=epoch)


class TestMmapQueueTimestamp:
    def test_roundtrip_preserves_timestamp(self):
        q = MmapJobQueue(maxsize=4, max_channels=4, max_samples=8)
        ts = 1_700_000_000_123_456
        q.put_nowait(_make_job(0, 1, ts))
        job = q.get(timeout=1.0)
        assert job is not None
        assert job.epoch.timestamp_us == ts
        q.close()

    def test_zero_timestamp_roundtrips(self):
        q = MmapJobQueue(maxsize=4, max_channels=4, max_samples=8)
        q.put_nowait(_make_job(0, 0, 0))
        job = q.get(timeout=1.0)
        assert job is not None
        assert job.epoch.timestamp_us == 0
        q.close()

    def test_poison_pill_still_works(self):
        q = MmapJobQueue(maxsize=4, max_channels=4, max_samples=8)
        q.put_nowait(None)
        job = q.get(timeout=1.0)
        assert job is None
        q.close()
