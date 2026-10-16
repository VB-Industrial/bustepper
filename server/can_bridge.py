#!/usr/bin/env python3
"""Run on Raspberry Pi in /opt/yakut/venv. CAN RPC only; never arms on startup."""
import asyncio
import math
import time
import uuid
from collections import deque
from aiohttp import web, WSMsgType
from pathlib import Path
from urllib.parse import urlsplit
import pycyphal.dsdl
from pycyphal.presentation import Presentation
from pycyphal.transport.can import CANTransport
from pycyphal.transport.can.media.socketcan import SocketCANMedia
from uavcan.register import Access_1, Name_1, Value_1
from uavcan.primitive.array import Integer32_1, Real64_1
from reg.udral.physics.kinematics.rotation import Planar_0_1

SETTINGS = ('ihold', 'irun', 'global_scaler', 'gear_ratio', 'acceleration',
            'vel_set', 'command_timeout_ms')
WRITABLE = set(SETTINGS) | {'arm', 'stop', 'move_rad', 'move', 'pos_set', 'tmc_pos_set', 'zero_set'}
INTEGER = {'ihold', 'irun', 'global_scaler', 'command_timeout_ms', 'arm', 'stop', 'tmc_pos_set', 'zero_set'}
ERRORS = {1: 'Неверное значение или диапазон', 2: 'Драйвер выключен',
          3: 'Требуется остановка или выключение', 4: 'Ошибка SPI / идентификации TMC'}

class Bridge:
    def __init__(self, presentation=None):
        self.presentation = presentation or Presentation(CANTransport(SocketCANMedia('can0', 64), 101))
        self.client = self.presentation.make_client_with_fixed_service_id(Access_1, 42)
        self.client.response_timeout = 0.4
        self.lock = asyncio.Lock()
        self.data = {'connected': False, 'settings': {}, 'error': 'Ожидание CAN', 'sample_id': 0, 'instance': str(uuid.uuid4())}
        self.last_settings = 0.0
        self.last_feedback = 0.0
        self.samples = deque(maxlen=400)
        self.sequences = {}
        self.subscriber = self.presentation.make_subscriber(Planar_0_1, 1042)
        self.subscriber.receive_in_background(self.on_feedback)
        self.started = time.monotonic()

    async def access(self, name, number=None):
        value = Value_1()
        if number is not None:
            value = (Value_1(integer32=Integer32_1(value=[int(number)])) if name in INTEGER
                     else Value_1(real64=Real64_1(value=[number])))
        result = await self.client.call(Access_1.Request(name=Name_1(name=name.encode()), value=value))
        if result is None:
            raise TimeoutError(f'Нет ответа узла 42: {name}')
        raw = pycyphal.dsdl.to_builtin(result[0].value)
        selected = next(iter(raw.values()))
        elements = selected.get('value', [])
        if not elements:
            raise ValueError(f'Пустой ответ регистра {name}')
        return elements

    async def on_feedback(self, message, transfer):
        if transfer.source_node_id != 42:
            return
        position = float(message.angular_position.radian)
        velocity = float(message.angular_velocity.radian_per_second)
        if not math.isfinite(position) or not math.isfinite(velocity):
            return
        now = time.monotonic()
        self.last_feedback = now
        sample_id = self.data['sample_id'] + 1
        sample = {'id': sample_id, 'time': now - self.started,
                  'position': position, 'velocity': velocity}
        self.samples.append(sample)
        self.data.update(position=position, velocity=velocity, sample_id=sample_id,
                         sample_time=sample['time'])

    async def poll(self):
        # Only service/settings use RPC. Motion feedback is a Cyphal subscription.
        while True:
            start = time.monotonic()
            try:
                async with self.lock:
                    status, steps = await asyncio.gather(self.access('status'), self.access('pos_steps'))
                    settings = self.data['settings']
                    if start - self.last_settings > 1 or not settings:
                        values = await asyncio.gather(*(self.access(name) for name in SETTINGS))
                        settings = {name: value[0] for name, value in zip(SETTINGS, values)}
                        self.last_settings = time.monotonic()
                    self.data.update(status=status, steps=steps[0], settings=settings,
                                     error=None, poll_ms=round((time.monotonic()-start)*1000,1))
            except Exception as exc:
                self.data['error'] = str(exc)
            await asyncio.sleep(max(0.01, 0.5 - (time.monotonic() - start)))

    def snapshot(self, since=-1):
        age = time.monotonic() - self.last_feedback
        connected = age < 0.5 and self.data.get('error') is None and bool(self.data.get('status'))
        return {**self.data, 'connected': connected,
                'feedback_age_ms': round(age*1000) if self.last_feedback else None,
                'feedback_subject': 1042, 'feedback_hz': 20,
                'samples': [s for s in self.samples if s['id'] > since]}

    async def telemetry(self, request):
        try:
            since = int(request.query.get('since', '-1'))
        except ValueError:
            since = -1
        return web.json_response(self.snapshot(since), headers={'Cache-Control': 'no-store'})

    async def execute(self, body):
        try:
            name, number = body.get('name'), body.get('value')
            session, sequence = body.get('session'), body.get('sequence')
            if not isinstance(session, str) or len(session) > 64 or not isinstance(sequence, int) or sequence < 1:
                raise ValueError('Требуется идентификатор команды')
            if name not in WRITABLE or isinstance(number, bool) or not isinstance(number, (int, float)) or not math.isfinite(number):
                raise ValueError('Недопустимая команда')
            if name in INTEGER and (number != int(number) or not -2147483648 <= number <= 2147483647):
                raise ValueError('Ожидается целое int32')
            if name in ('ihold', 'irun') and not 0 <= number <= 31:
                raise ValueError('Ток: допустимы 0…31')
            if name == 'global_scaler' and number != 0 and not 32 <= number <= 255:
                raise ValueError('GLOBAL_SCALER: допустимы 0 или 32…255')
            if name == 'arm' and number not in (0, 1):
                raise ValueError('arm: 0 или 1')
            async with self.lock:
                if sequence <= self.sequences.get(session, 0):
                    raise ValueError('Устаревшая команда отклонена')
                self.sequences[session] = sequence
                if len(self.sequences) > 128:
                    self.sequences.pop(next(iter(self.sequences)))
                # Enforce watchdog for velocity control even if changed manually in Yakut.
                if name in ('move_rad', 'move'):
                    await self.access('command_timeout_ms', 3000)
                response = await self.access(name, number)
                status = await self.access('status')
                if status[2]:
                    raise ValueError(ERRORS.get(status[2], f'Ошибка привода {status[2]}'))
                if name in SETTINGS:
                    self.data['settings'][name] = response[0]
                self.last_settings = 0
                # Reject a nominally successful write if the motor did not adopt arm state.
                if name == 'arm' and response[0] != number:
                    raise ValueError('Привод не подтвердил включение/выключение')
            return {'ok': True, 'name': name, 'value': response, 'status': status}
        except (ValueError, TimeoutError, TypeError, AttributeError) as exc:
            return {'ok': False, 'error': str(exc)}

    async def command(self, request):
        result = await self.execute(await request.json())
        return web.json_response(result, status=200 if result['ok'] else 400)

    async def websocket(self, request):
        socket = web.WebSocketResponse(heartbeat=10, max_msg_size=4096)
        await socket.prepare(request)
        async def publish():
            since = -1
            while not socket.closed:
                data = self.snapshot(since)
                since = data['sample_id']
                await socket.send_json({'type': 'telemetry', **data})
                await asyncio.sleep(0.05)
        publisher = asyncio.create_task(publish())
        try:
            async for message in socket:
                if message.type == WSMsgType.TEXT:
                    try:
                        body = message.json()
                        if not isinstance(body, dict):
                            raise ValueError('Ожидается объект команды')
                        if body.get('type') != 'command':
                            raise ValueError('Неизвестный тип сообщения')
                        result = await self.execute(body)
                        await socket.send_json({'type': 'command_result',
                                                'sequence': body.get('sequence'), **result})
                    except (ValueError, TypeError) as exc:
                        await socket.send_json({'type': 'command_result', 'ok': False, 'error': str(exc)})
        finally:
            publisher.cancel()
            await asyncio.gather(publisher, return_exceptions=True)
        return socket

@web.middleware
async def same_origin(request, handler):
    origin = request.headers.get('Origin')
    if origin and urlsplit(origin).netloc != request.host:
        raise web.HTTPForbidden(text='Cross-origin control is disabled')
    if request.path == '/api/command' and request.method == 'POST':
        if request.headers.get('X-Bustepper') != '1' or request.content_type != 'application/json':
            raise web.HTTPForbidden()
    return await handler(request)

def application(bridge, ui_dir=None):
    app = web.Application(client_max_size=4096, middlewares=[same_origin])
    app.router.add_get('/api/state', bridge.telemetry)
    app.router.add_post('/api/command', bridge.command)
    app.router.add_get('/ws', bridge.websocket)
    if ui_dir is not None:
        async def index(request):
            return web.FileResponse(ui_dir / 'index.html', headers={'Cache-Control': 'no-cache'})
        app.router.add_get('/', index)
        app.router.add_static('/', ui_dir, show_index=False)
    return app

async def main():
    bridge = Bridge()
    ui_dir = Path(__file__).resolve().parent / 'ui'
    if not ui_dir.is_dir():
        ui_dir = Path(__file__).resolve().parents[1] / 'ui'
    runner = web.AppRunner(application(bridge, ui_dir))
    await runner.setup()
    await web.TCPSite(runner, '0.0.0.0', 8088).start()
    print('BUStepper HTTP + WebSocket: :8088; CAN0 node 101 -> drive 42', flush=True)
    task = asyncio.create_task(bridge.poll())
    try:
        await asyncio.Event().wait()
    finally:
        task.cancel()
        await asyncio.gather(task, return_exceptions=True)
        bridge.presentation.close()
        await runner.cleanup()

if __name__ == '__main__':
    asyncio.run(main())
