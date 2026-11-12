"""One-shot explicit START capture owner using the existing Async UART only."""
from concurrent.futures import Future
from dataclasses import asdict
from pathlib import Path
import threading
from sentinel_short_capture import ShortCaptureAdapter


class PositioningCaptureOwner:
    def __init__(self, peer):
        self.peer = peer
        self.settings = None
        self.adapter = None
        self.receipt = None
        self._lock = threading.Lock()

    def configure(self, settings):
        if self.adapter is not None:
            raise ValueError('Capture already attempted; no implicit replacement/reset')
        if set(settings) != {'run', 'scheduling_margin_us', 'output_path'}:
            raise ValueError('capture requires run / scheduling_margin_us / output_path')
        for key, upper in (('run', 0xffffffff), ('scheduling_margin_us', 255999)):
            if type(settings[key]) is not int or not 0 < settings[key] <= upper:
                raise ValueError('Explicit positive capture ' + key + ' required')
        path = Path(settings['output_path'])
        if path.suffix != '.sraw' or path.exists():
            raise ValueError('New exclusive .sraw output path required')
        if not path.parent.is_dir():
            raise ValueError('Capture output parent directory must already exist')
        self.settings = dict(settings)

    def start(self, recipe, snapshot):
        with self._lock:
            if self.settings is None or self.adapter is not None:
                raise ValueError('Load explicit capture settings; one attempt only')
            identities = {key: snapshot.get(key) for key in
                ('session', 'boot_id', 'firmware_sha256', 'deployment_sha256',
                 'config_epoch', 'reference_epoch', 'param_epoch', 'config_hash',
                 'guardian_version', 'experiment_session', 'nonce', 'axis')}
            if any(value is None for value in identities.values()) or not identities['boot_id']:
                raise ValueError('Missing observed capture identity')
            if self.peer.snapshot().get('session') != identities['session']:
                raise ValueError('Capture session changed')
            bound_session = identities['session']
            def exchange(payload):
                if self.peer.snapshot().get('session') != bound_session:
                    raise ValueError('Capture session changed; no raw request sent')
                return self.peer.raw_request(payload, expected_session=bound_session)
            self.adapter = ShortCaptureAdapter(exchange,
                scheduling_margin_us=self.settings['scheduling_margin_us'], identities=identities)
            future = self.adapter.prepare_start(5 if recipe['axis']=='fire_yaw' else 6,
                self.settings['run'], recipe['duration_us'])
        result = Future()
        def completed(done):
            try:
                receipt = done.result()
                self.receipt = receipt
                if not result.done():
                    result.set_result(asdict(receipt))
            except Exception as exc:
                if not result.done():
                    result.set_exception(exc)
        future.add_done_callback(completed)
        return result

    def finish(self):
        if self.adapter is None or self.receipt is None:
            raise ValueError('No acknowledged finite capture to export')
        return self.adapter.finish(self.settings['output_path'])

    def close(self):
        if self.adapter is not None:
            self.adapter.close(wait=False)  # worker only; no motion/logging STOP implied
