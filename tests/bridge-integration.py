"""Exercise real PyCyphal RPC and pub/sub over loopback UDP, never physical CAN."""
import asyncio
from collections import Counter
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import pycyphal.dsdl
from pycyphal.presentation import Presentation
from pycyphal.transport.udp import UDPTransport
from aiohttp import web
from aiohttp.test_utils import TestClient, TestServer
from uavcan.register import Access_1, Value_1
from uavcan.primitive.array import Integer32_1, Real64_1
from reg.udral.physics.kinematics.rotation import Planar_0_1
from server.can_bridge import Bridge, application

async def main():
    motor = Presentation(UDPTransport('127.0.0.1', 42))
    bridge = Bridge(Presentation(UDPTransport('127.0.0.1', 101)))
    reads, writes = Counter(), []
    regs = {'ihold': 0, 'irun': 0, 'global_scaler': 33, 'gear_ratio': 10.0,
            'acceleration': 0.5, 'vel_set': 0.25, 'command_timeout_ms': 3000,
            'arm': 0, 'move_rad': 0.0, 'pos_set': 0.0}
    mode, error = 0, 0
    async def access(request, metadata):
        nonlocal mode, error
        name = bytes(request.name.name).decode()
        incoming = pycyphal.dsdl.to_builtin(request.value)
        if 'empty' not in incoming:
            number = next(iter(incoming.values()))['value'][0]
            writes.append((name, number))
            if name == 'irun' and number == 31:
                error = 1  # Simulate firmware rejecting a setting.
            else:
                error = 0
                regs[name] = number
                if name == 'move_rad': mode = 1
                if name == 'stop': mode = 0
        else:
            reads[name] += 1
        if name == 'status':
            value = Value_1(integer32=Integer32_1(value=[regs['arm'], mode, error, 100, 0, 0]))
        elif name == 'pos_steps':
            value = Value_1(integer32=Integer32_1(value=[101859]))
        elif isinstance(regs[name], int):
            value = Value_1(integer32=Integer32_1(value=[regs[name]]))
        else:
            value = Value_1(real64=Real64_1(value=[regs[name]]))
        return Access_1.Response(mutable=name not in ('status', 'pos_steps'), value=value)
    motor.get_server_with_fixed_service_id(Access_1).serve_in_background(access)
    publisher = motor.make_publisher(Planar_0_1, 1042)
    message = Planar_0_1()
    message.angular_position.radian = 1.25
    message.angular_velocity.radian_per_second = -0.05
    async def publish():
        while True:
            await publisher.publish(message)
            await asyncio.sleep(0.05)
    poll = asyncio.create_task(bridge.poll())
    pub = asyncio.create_task(publish())
    app = application(bridge)
    client = TestClient(TestServer(app), headers={'X-Bustepper':'1'})
    await client.start_server()
    try:
        for _ in range(40):
            state = await (await client.get('/api/state')).json()
            if state['connected'] and state['sample_id'] >= 3:
                break
            await asyncio.sleep(0.1)
        assert state['connected'], state
        assert abs(state['position'] - 1.25) < 1e-6
        assert abs(state['velocity'] + 0.05) < 1e-6
        assert not writes, 'Startup must be read-only'
        assert not reads['pos_get'] and not reads['velocity'], reads
        assert state['feedback_subject'] == 1042
        for name, number, sequence in [('irun', 5, 1), ('arm', 1, 2), ('move_rad', 0.1, 3), ('stop', 1, 4)]:
            response = await client.post('/api/command', json={'session':'test','sequence':sequence,'name':name,'value':number})
            assert response.status == 200, await response.text()
        assert ('command_timeout_ms', 3000) in writes
        before = len(writes)
        response = await client.post('/api/command', json={'session':'test','sequence':3,'name':'move_rad','value':1})
        assert response.status == 400 and len(writes) == before, 'Late motion command must be rejected'
        response = await client.post('/api/command', json={'session':'test','sequence':5,'name':'irun','value':31})
        assert response.status == 400, 'Firmware rejection must reach UI'
        response = await client.post('/api/command', json={'session':'test','sequence':6,'name':'global_scaler','value':1})
        assert response.status == 400
        ws = await client.ws_connect('/ws')
        frame = await ws.receive_json()
        assert frame['type'] == 'telemetry' and frame['connected']
        await ws.send_json({'type':'command','session':'test','sequence':7,'name':'irun','value':6})
        while True:
            frame = await ws.receive_json()
            if frame['type'] == 'command_result':
                assert frame['ok'] and frame['sequence'] == 7
                break
        assert ('irun',6) in writes
        await ws.close()
        response = await client.post('/api/command', headers={'Origin':'http://unrelated.invalid'},
                                     json={'session':'test','sequence':8,'name':'arm','value':1})
        assert response.status == 403
        pub.cancel()
        await asyncio.gather(pub, return_exceptions=True)
        await asyncio.sleep(0.7)
        state = await (await client.get('/api/state')).json()
        assert not state['connected'], 'Missing feedback must disable movement'
        print('Bridge HTTP/WebSocket integration passed: real DSDL pub/sub and register RPC on loopback; no telemetry RPC; startup read-only; stale-command and firmware-error handling.')
    finally:
        poll.cancel(); pub.cancel()
        await asyncio.gather(poll, pub, return_exceptions=True)
        await client.close()
        bridge.presentation.close(); motor.close()

asyncio.run(main())
