"""Slope scene mocks only: no CARLA server, CAN or network connections."""
import math
import json
from pathlib import Path
import struct
import sys
import types
import unittest
from unittest.mock import patch, Mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'carla_tools'))
from test_carla_tools import Actor, TF, WP, FakeWorld, fake_carla
import carla_hmi_bridge as bridge
from hmi import scene
from PySide6.QtTest import QTest


class SlopeWP(WP):
    def __init__(self, s, grade, end=None):
        super().__init__(s)
        self.grade, self.end = grade, end
        self.transform.location.z = 10 + s * grade

    def next(self, d):
        return [] if self.end is not None and self.s >= self.end else [SlopeWP(self.s + d, self.grade, self.end)]

    def previous(self, d):
        return [SlopeWP(self.s - d, self.grade, self.end)]


class SlopeMap:
    name = 'MockSlope'

    def __init__(self, grade, end=None):
        self.grade, self.end = grade, end

    def get_waypoint(self, *args, **kwargs):
        return SlopeWP(0, self.grade, self.end)


def packet(grade=0, pitch=0, roll=0, end=None, ego_pitch=None):
    ego = Actor(1, 'vehicle.ego', 0, 0, 90)
    ego._tf.location.z = 10
    ego._tf.rotation.pitch = math.degrees(math.atan(grade)) if ego_pitch is None else ego_pitch
    actors = [ego]
    for aid, kind in ((2, 'vehicle.other'), (3, 'walker.pedestrian.0001')):
        a = Actor(aid, kind, 0, 20, 90)
        a._tf.location.z = 10 + 20 * grade
        a._tf.rotation.pitch, a._tf.rotation.roll = pitch, roll
        actors.append(a)
    args = types.SimpleNamespace(ahead=80, behind=20, side=20)
    return bridge.build_packet(fake_carla, FakeWorld(actors), SlopeMap(grade, end), ego, args, 1)


def build(pkt):
    return scene.build_scene(scene.SpriteLibrary('/nonexistent'), carla=pkt)


class SlopePacketTest(unittest.TestCase):
    def test_flat_uphill_downhill_and_legacy_algorithms(self):
        flat = packet()
        for grade in (0, math.tan(math.radians(6)), math.tan(math.radians(-6))):
            with self.subTest(grade=grade):
                pitch = math.degrees(math.atan(grade))
                pkt = packet(grade, pitch)
                self.assertEqual(pkt['ego']['z'], 10)
                self.assertTrue(all(abs(p[2]) < .001 for p in pkt['lane']['center3d']))
                lane30 = min(pkt['lane']['center3d'], key=lambda p: abs(p[0] - 30 / math.cos(math.radians(pitch))))
                self.assertAlmostEqual(lane30[0], 30 / math.cos(math.radians(pitch)), places=3)
                self.assertEqual(pkt['lane']['center'], flat['lane']['center'])
                self.assertEqual(pkt['lead'], flat['lead'])
                sc = build(pkt)
                for v in sc['vehicles']:
                    self.assertAlmostEqual(v['local_x'], 20 / math.cos(math.radians(pitch)), places=3)
                    self.assertEqual((v['local_y'], v['local_z'], v['local_pitch'], v['local_roll']), (0, 0, 0, 0))
                    self.assertEqual((v['x'], v['y']), (20, 0))
                    self.assertEqual((v['relative_pitch'], v['relative_roll'], v['relative_yaw']), (0, 0, 0))
                self.assertTrue(sc['risk_zone3d'])
                self.assertEqual(sc['risk_zone'], build(flat)['risk_zone'])
                self.assertTrue(all(abs(p[2]) < .001 for p in sc['risk_zone3d']))

    def test_road_grade_world_samples_and_degenerate_segment(self):
        for angle in (0, 6, -6):
            with self.subTest(angle=angle):
                wp = SlopeWP(0, math.tan(math.radians(angle)))
                # Mock road pitch is zero even on slopes: world height samples win.
                self.assertEqual(wp.transform.rotation.pitch, 0)
                self.assertAlmostEqual(bridge.road_grade_deg(wp, 90), angle, places=3)
                self.assertAlmostEqual(bridge.road_grade_deg(wp, -90), -angle, places=3)
                # Only rear segment available at end of road.
                wp.next = lambda distance: []
                self.assertAlmostEqual(bridge.road_grade_deg(wp, 90), angle, places=3)
                # Isolated waypoint: no horizontal segment, publish 0.0.
                wp.previous = lambda distance: []
                wp.transform.rotation.pitch = angle
                self.assertEqual(bridge.road_grade_deg(wp, 90), 0.0)
                self.assertEqual(bridge.road_grade_deg(wp, -90), 0.0)

    def test_top_level_grade_packet_json_log_and_precedence(self):
        for angle in (5.83, -5.83, 0):
            with self.subTest(angle=angle):
                pkt = packet(math.tan(math.radians(angle)), angle, ego_pitch=.1)
                assert "road_grade_deg" in pkt
                decoded = json.loads(json.dumps(pkt))
                self.assertEqual(decoded["road_grade_deg"], angle)
                # The final packet, not lane data or an independent log calculation, wins.
                decoded['lane']['road_grade_deg'] = 99
                self.assertEqual(build(decoded)['road_grade_deg'], angle)
                with patch('builtins.print') as log:
                    bridge.debug_scene_packet(decoded)
                self.assertIn('roadGrade=' + str(decoded['road_grade_deg']), log.call_args.args[0])
        with patch('carla_hmi_bridge.lane_info', return_value=None):
            pkt = packet(ego_pitch=6)
        self.assertIn('road_grade_deg', pkt)
        self.assertEqual(pkt['road_grade_deg'], 0.0)

    def test_full_transform_flat_and_constant_six_degree_grades(self):
        ego = TF(0, 0, 0)
        ego.location.z = 10
        for angle in (0, 6, -6):
            ego.rotation.pitch = angle
            a = math.radians(angle)
            # Horizontal separation 30m; along-slope distance is 30/cos(6).
            actual = bridge.to_ego_3d(ego, 30, 0, 10 + 30 * math.tan(a))
            for value, expected in zip(actual, (30 / math.cos(a), 0, 0)):
                self.assertAlmostEqual(value, expected, places=10)
            # Along-slope separation exactly 30m.
            actual = bridge.to_ego_3d(ego, 30 * math.cos(a), 0, 10 + 30 * math.sin(a))
            for value, expected in zip(actual, (30, 0, 0)):
                self.assertAlmostEqual(value, expected, places=10)

    def test_change_from_six_to_ten_degree_grade(self):
        ego = TF(0, 0, 0)
        ego.location.z = 10
        ego.rotation.pitch = 6
        z = bridge.to_ego_3d(ego, 30, 0, 10 + 30 * math.tan(math.radians(10)))[2]
        expected = 30 * math.cos(math.radians(6)) * (math.tan(math.radians(10)) - math.tan(math.radians(6)))
        self.assertAlmostEqual(z, expected, places=10)
        self.assertGreater(z, 0)
        self.assertLess(z, 30 * math.tan(math.radians(10)))

    def test_translation_yaw_pitch_roll_and_relative_rotation(self):
        ego = TF(7, -13, 37)
        ego.location.z = 10
        ego.rotation.pitch, ego.rotation.roll = 6, -4
        m = ego.get_matrix()
        local = (30, 2, -1, 1)
        world = [sum(m[i][j]*local[j] for j in range(4)) for i in range(3)]
        actual = bridge.to_ego_3d(ego, *world)
        for value, expected in zip(actual, local):
            self.assertAlmostEqual(value, expected, places=10)
        self.assertTrue(all(abs(a) < 1e-10 for a in bridge.relative_rotation(ego.get_inverse_matrix(), ego)))

    def test_relative_rotation_composition_matches_qt_basis(self):
        from PySide6.QtGui import QQuaternion, QVector3D
        ego = TF(7, -13, 37)
        ego.rotation.pitch, ego.rotation.roll = 6, -4
        inv = ego.get_inverse_matrix()
        for yaw, pitch, roll in ((37, 6, -4), (-15, 10, 3), (170, -5, -7), (37, 90, 0)):
            actor = TF(0, 0, yaw)
            actor.rotation.pitch, actor.rotation.roll = pitch, roll
            ry, rp, rr = bridge.relative_rotation(inv, actor)
            q = QQuaternion.fromEulerAngles(rp, -ry, -rr)
            matrix = actor.get_matrix()
            for col, basis in ((0, QVector3D(0, 0, -1)), (1, QVector3D(1, 0, 0)), (2, QVector3D(0, 1, 0))):
                local = [sum(inv[i][k] * matrix[k][col] for k in range(3)) for i in range(3)]
                expected = (local[1], local[2], -local[0])
                actual = q.rotatedVector(basis)
                for value, target in zip((actual.x(), actual.y(), actual.z()), expected):
                    self.assertAlmostEqual(value, target, places=6)

    def test_lane_extension_stays_in_full_local_frame(self):
        pkt = packet(math.tan(math.radians(6)), end=4)
        self.assertGreater(pkt['lane']['center'][-1][0], 40)
        self.assertTrue(all(abs(p[2]) < .001 for p in pkt['lane']['center3d']))
        self.assertGreater(pkt['lane']['center3d'][-1][0], 40)

    def test_old_packet_and_flat_are_equivalent(self):
        pkt = packet()
        flat = build(pkt)
        del pkt['lane']['center3d']
        for obj in [pkt['ego']] + pkt['vehicles']:
            for field in ('z', 'pitch', 'roll'):
                obj.pop(field)
            for field in ('local_x', 'local_y', 'local_z', 'local_yaw', 'local_pitch', 'local_roll', 'relative_yaw', 'relative_pitch', 'relative_roll'):
                obj.pop(field, None)
        old = build(pkt)
        for v in flat['vehicles']:
            for field in ('local_x', 'local_y', 'local_z', 'local_yaw', 'local_pitch', 'local_roll', 'relative_yaw', 'relative_pitch', 'relative_roll'):
                v.pop(field)
        # Geometry compatibility; trace/mode metadata intentionally identifies fallback.
        self.assertEqual(flat['center3d'], old['center3d'])
        for key in ('lines3d', 'road_left3d', 'road_right3d', 'vehicles', 'risk_zone'):
            self.assertEqual(flat[key], old[key])
        self.assertFalse(old['slope_scene'])
        self.assertTrue(all(p[2] == 0 for p in old['center3d']))

    def test_center3d_preferred_and_invalid_optional_falls_back(self):
        pkt = packet(.1)
        pkt['lane']['center'] = [[-10, 0], [60, 0]]
        pkt['lane']['center3d'] = [[-10, 0, -1], [60, 0, 6]]
        self.assertEqual(scene.interp_z(build(pkt)['center3d'], 20), 2)
        for value in ([[0, 0, float('nan')], [20, 0, 2]], [[0, 0], [20, 0]], 'bad'):
            pkt['lane']['center3d'] = value
            self.assertTrue(all(p[2] == 0 for p in build(pkt)['center3d']))


class SlopeQtTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from PySide6.QtGui import QGuiApplication
        from hmi.geometry3d import register
        cls.app = QGuiApplication.instance() or QGuiApplication(['slope', '-platform', 'offscreen'])
        register()

    def test_mesh_height_normals_bounds_and_old_2d(self):
        from hmi.geometry3d import RibbonGeometry
        for grade in (0, .1, -.1):
            g = RibbonGeometry()
            g.points = [[0, 0, 0], [20, 0, 20 * grade]]
            verts = [struct.unpack_from('<8f', bytes(g.vertexData()), i) for i in range(0, len(g.vertexData()), 32)]
            for x, y, z, nx, ny, nz, u, v in verts:
                self.assertAlmostEqual(y, -z * grade + g.lift, places=5)
                self.assertGreater(ny, 0)
                self.assertAlmostEqual(nz / ny, grade, places=5)
                self.assertLessEqual(g.boundsMin().y(), y + 1e-6)
                self.assertGreaterEqual(g.boundsMax().y(), y - 1e-6)
            g.left = [[0, -1, 0], [20, -1, 20 * grade]]
            g.right = [[0, 1, 0], [20, 1, 20 * grade]]
            self.assertEqual(len(g.vertexData()), 6 * 32)
        old, new = RibbonGeometry(), RibbonGeometry()
        old.points = [[0, 0], [20, 0]]
        new.points = [[0, 0, 0], [20, 0, 0]]
        self.assertEqual(bytes(old.vertexData()), bytes(new.vertexData()))

    def test_actual_world_qml_transform_and_flat_occluders(self):
        from PySide6.QtCore import QUrl, QObject
        from PySide6.QtGui import QVector3D
        from PySide6.QtQml import QQmlComponent, QQmlEngine
        engine = QQmlEngine()
        component = QQmlComponent(engine, QUrl.fromLocalFile(str(ROOT / 'qml/World3D.qml')))
        self.assertFalse(component.errors(), component.errors())
        world = component.create()
        self.assertIsNotNone(world)
        for grade, pitch, roll, yaw in ((.1, 5, 3, 0), (-.1, -5, -3, 35), (.1, 5, -3, 180), (0, 0, 0, 0)):
            pkt = packet(grade, pitch, roll)
            for actor in pkt["vehicles"]:
                actor["relative_yaw"] = yaw
                actor["relative_pitch"] = pitch
                actor["relative_roll"] = roll
                actor["local_yaw"] = yaw
                actor["local_pitch"] = pitch
                actor["local_roll"] = roll
            pkt.update(map_ver=1, map_xf={'x': 0, 'y': 0, 'rot': 0})
            sc = build(pkt)
            world.setProperty('carlaMap', {'ver': 1})
            world.setProperty('sc', sc)
            self.app.processEvents()
            self.assertTrue(world.property('mapOn'))
            if grade == 0:
                QTest.qWait(550)
            self.assertEqual(world.property('mapRoadOn'), grade == 0)
            self.assertEqual(world.property('mapBoxesOn'), grade == 0)
            nodes = world.findChildren(QObject)
            traffic = [n for n in nodes if n.property('kind') in ('car', 'person') and n.property('modelData') is not None]
            self.assertEqual(len(traffic), 2)
            for n in traffic:
                self.assertAlmostEqual(n.property('position').y(), 0)
                self.assertAlmostEqual(-n.property('position').z(), sc['vehicles'][0]['local_x'], places=5)
                # Verify Qt's actual quaternion against CARLA's forward/right formulas.
                rotation = n.property('rotation')
                front = rotation.rotatedVector(QVector3D(0, 0, -1))
                right = rotation.rotatedVector(QVector3D(1, 0, 0))
                cp, sp = math.cos(math.radians(pitch)), math.sin(math.radians(pitch))
                cy, sy = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
                cr, sr = math.cos(math.radians(roll)), math.sin(math.radians(roll))
                # CARLA vectors transformed by (x,y,z) -> (y,z,-x).
                for actual, expected in zip((front.x(), front.y(), front.z()), (sy*cp, sp, -cy*cp)):
                    self.assertAlmostEqual(actual, expected, places=6)
                for actual, expected in zip((right.x(), right.y(), right.z()),
                                            (sy*sp*sr+cy*cr, -cp*sr, -cy*sp*sr+sy*cr)):
                    self.assertAlmostEqual(actual, expected, places=6)
                self.assertAlmostEqual(right.y(), -math.cos(math.radians(pitch))*math.sin(math.radians(roll)), places=6)
            ground = [n for n in nodes if n.property('position') == QVector3D(0, -.01, -60)]
            self.assertEqual(len(ground), 1)
            self.assertEqual(ground[0].property('visible'), grade == 0)
        # Compile the actual view and verify the camera does not follow ego tilt.
        view_component = QQmlComponent(engine, QUrl.fromLocalFile(str(ROOT / 'qml/DriveVis3D.qml')))
        self.assertFalse(view_component.errors(), view_component.errors())
        view = view_component.create()
        self.assertIsNotNone(view)
        view.setProperty('world', world)
        self.app.processEvents()
        cameras = [n for n in view.findChildren(QObject) if n.property('clipNear') is not None]
        self.assertEqual(len(cameras), 1)
        timers = [n for n in view.findChildren(QObject) if n.property('interval') == 500]
        self.assertEqual(len(timers), 1)
        self.assertTrue(timers[0].property('repeat'))
        cam = cameras[0]
        before = (cam.property('position'), cam.property('rotation'))
        for grade in (.1, -.1):
            world.setProperty('sc', build(packet(grade)))
            self.app.processEvents()
            self.assertEqual((cam.property('position'), cam.property('rotation')), before)
            self.assertEqual((cam.property('clipNear'), cam.property('clipFar')), (.5, 400))
        view.deleteLater()
        world.deleteLater()

    def test_json_receiver_backend_qml_end_to_end_and_debug(self):
        from PySide6.QtCore import QObject, QUrl, QMetaObject, Qt, Q_RETURN_ARG, QCoreApplication, QEvent
        from PySide6.QtGui import QVector3D
        from PySide6.QtQml import QQmlEngine, QQmlComponent
        from hmi.backend import HmiBackend
        from hmi.sources import UdpReceiver
        from hmi.geometry3d import RibbonGeometry
        with patch('hmi.backend.AlertSound') as sound:
            sound.return_value.muted = True
            sound.return_value.backend = 'mock'
            backend = HmiBackend(sound=False)
        backend._timer.stop()
        backend._diag_timer.stop()
        engine = QQmlEngine()
        engine.rootContext().setContextProperty('hmi', backend)
        component = QQmlComponent(engine)
        component.setData(b'import QtQuick\nimport QtQuick3D\nimport "."\nWorld3D { sc: hmi.state.scene || ({}); carlaMap: hmi.carlaMap }',
                          QUrl.fromLocalFile(str(ROOT / 'qml/TraceHarness.qml')))
        self.assertFalse(component.errors(), component.errors())
        world = component.create()
        self.assertIsNotNone(world)
        snapshots = []
        for angle in (0, 6, -6):
            pkt = packet(math.tan(math.radians(angle)), angle)
            pkt['seq'] = angle + 10
            del pkt['lane']['road_grade_deg']  # Wire/HMI path must work with top-level field alone.
            pkt.update(map_ver=4, map_xf={'x': 2, 'y': 3, 'rot': 12})
            # Poison legacy render values: assertions must fail if local priority breaks.
            for v in pkt['vehicles']:
                v.update(x=18, y=8, z=99, pitch=45, roll=35, local_pitch=50)
            ui = {'type': 'carla_ui_state', 'night_mode': True,
                  'lane_assist_enabled': True, 'lane_assist_active': True}
            raw = [json.dumps(p).encode() for p in (pkt, ui)]
            receiver = UdpReceiver(backend.sink, kind='carla', port=5601)
            sock = Mock()
            def receive(_size):
                payload = raw.pop(0)
                if not raw:
                    receiver.stop()
                return payload, ('mock', 0)
            sock.recvfrom.side_effect = receive
            # Run the real JSON receiver with an in-memory socket only.
            with patch('hmi.sources.socket.socket', return_value=sock), patch('hmi.sources.time.monotonic', return_value=100):
                receiver.run()
            backend._carla_map = {'ver': 4}
            backend.carlaMapChanged.emit()
            with patch('hmi.backend.time.monotonic', return_value=100):
                backend._tick()
            self.app.processEvents()
            QCoreApplication.sendPostedEvents(None, QEvent.DeferredDelete)
            received = backend._carla_store['data']
            self.assertEqual(received, pkt)
            self.assertTrue(backend.state['night_mode'])
            self.assertTrue(backend.state['lane_assist_active'])
            self.assertEqual(backend.state['scene']['debug3d']['received_local'], 2)
            traffic = [n for n in world.findChildren(QObject)
                       if n.property('positionMode') is not None]
            self.assertEqual(len(traffic), 2)
            for n in traffic:
                local_x = pkt['vehicles'][0]['local_x']
                self.assertEqual(n.property('positionMode'), 'local')
                self.assertEqual(n.property('rotationMode'), 'relative')
                self.assertEqual(n.property('position'), QVector3D(0, world.property('vehicleHeightOffset'), -local_x))
                self.assertEqual(n.property('eulerRotation'), QVector3D(0, 0, 0))
                self.assertEqual(n.property('scenePosition'), n.property('position'))
            # Check the geometry created by actual World3D bindings, not a separate mesh.
            roads = [g for g in world.findChildren(RibbonGeometry) if len(g.left) >= 2 and len(g.right) >= 2]
            self.assertTrue(roads)
            road = roads[0]
            self.assertEqual(road.left, backend.state['scene']['road_left3d'])
            data = bytes(road.vertexData())
            ys = [struct.unpack_from('<8f', data, i)[1] for i in range(0, len(data), 32)]
            self.assertTrue(all(abs(y) < 1e-6 for y in ys))
            self.assertTrue(all(abs(p[2]) < .001 for p in received['lane']['center3d'] if 20 <= p[0] <= 31))
            ego = world.findChild(QObject, 'ego3d')
            self.assertIsNotNone(ego)
            self.assertEqual(ego.property('position'), QVector3D(0, 0, 0))
            self.assertEqual(ego.property('eulerRotation'), QVector3D(0, 0, 0))
            snapshots.append((world.property('position'), world.property('eulerRotation')))
            debug = QMetaObject.invokeMethod(world, 'debugLine', Qt.DirectConnection, Q_RETURN_ARG("QVariant"))
            self.assertIn('stage=qml', debug)
            self.assertIn('frame=ego_local_3d_v1', debug)
            self.assertIn('vehicleMode=local/relative', debug)
            self.assertIn('rawLocal=2', debug)
            self.assertIn('mapRoadVisible=' + str(int(angle == 0)), debug)
            self.assertIn('sceneRoad3DVisible=' + str(int(angle != 0)), debug)
            self.assertNotIn('final=missing', debug)
        self.assertTrue(all(s == snapshots[0] for s in snapshots))
        world.deleteLater()
        QCoreApplication.sendPostedEvents(None, QEvent.DeferredDelete)
        backend.deleteLater()

    def test_nonzero_local_height_reaches_actual_model_and_mesh_once(self):
        from PySide6.QtCore import QObject, QUrl, QCoreApplication, QEvent
        from PySide6.QtGui import QVector3D
        from PySide6.QtQml import QQmlEngine, QQmlComponent
        from hmi.geometry3d import RibbonGeometry
        engine = QQmlEngine()
        c = QQmlComponent(engine, QUrl.fromLocalFile(str(ROOT / 'qml/World3D.qml')))
        world = c.create()
        for height in (.4, -.4):
            pkt = packet(math.tan(math.radians(6)), 6)
            pkt['ego']['z'] = 10000  # Must never move ego/root or retransform local geometry.
            pkt['lane']['center3d'] = [[0, 0, 0], [25, 1, height], [50, 2, height * 2]]
            for v in pkt['vehicles']:
                v.update(local_x=25, local_y=1, local_z=height, x=18, y=9, z=99)
            world.setProperty('sc', build(pkt))
            self.app.processEvents()
            QCoreApplication.sendPostedEvents(None, QEvent.DeferredDelete)
            traffic = [n for n in world.findChildren(QObject) if n.property('positionMode') == 'local']
            self.assertEqual(len(traffic), 2)
            for n in traffic:
                self.assertEqual(n.property('scenePosition'), QVector3D(1, height + world.property('vehicleHeightOffset'), -25))
            road = next(g for g in world.findChildren(RibbonGeometry) if len(g.left) >= 2 and len(g.right) >= 2)
            data = bytes(road.vertexData())
            verts = [struct.unpack_from('<8f', data, i) for i in range(0, len(data), 32)]
            lane_point = road.left[1]
            self.assertAlmostEqual(lane_point[2], height)
            self.assertTrue(any(abs(p[0] - lane_point[1]) < 1e-5 and abs(p[2] + lane_point[0]) < 1e-5
                                and abs(p[1] - height) < 1e-6 for p in verts))
            ego = world.findChild(QObject, 'ego3d')
            self.assertEqual(ego.property('scenePosition'), QVector3D(0, 0, 0))
            self.assertEqual(world.property('scenePosition'), QVector3D(0, 0, 0))
        world.deleteLater()

    def test_relative_names_only_and_local_position_without_absolute_rotation(self):
        from PySide6.QtCore import QObject, QUrl
        from PySide6.QtGui import QVector3D
        from PySide6.QtQml import QQmlEngine, QQmlComponent
        engine = QQmlEngine()
        c = QQmlComponent(engine, QUrl.fromLocalFile(str(ROOT / 'qml/World3D.qml')))
        world = c.create()
        pkt = packet(math.tan(math.radians(6)), 6)
        for v in pkt['vehicles']:
            for key in ('local_yaw', 'local_pitch', 'local_roll'):
                v.pop(key)
            v.update(pitch=80, roll=70)
        world.setProperty('sc', build(pkt))
        self.app.processEvents()
        for n in world.findChildren(QObject):
            if n.property('positionMode') == 'local':
                self.assertEqual(n.property('eulerRotation'), QVector3D(0, 0, 0))
        # If relative rotation is absent in a local position packet, use neutral
        # pitch/roll, never the absolute 80/70 degree actor values.
        for v in pkt['vehicles']:
            for key in ('relative_pitch', 'relative_roll', 'relative_yaw'):
                v.pop(key)
        world.setProperty('sc', build(pkt))
        self.app.processEvents()
        for n in world.findChildren(QObject):
            if n.property('positionMode') == 'local':
                self.assertEqual(n.property('eulerRotation'), QVector3D(0, 0, 0))
        world.deleteLater()

    def test_flat_map_layers_and_slope_hysteresis(self):
        from PySide6.QtCore import QObject, QUrl, QMetaObject, Qt, Q_RETURN_ARG
        from PySide6.QtGui import QVector3D
        from PySide6.QtQml import QQmlEngine, QQmlComponent
        from hmi.geometry3d import RibbonGeometry
        engine = QQmlEngine()
        c = QQmlComponent(engine, QUrl.fromLocalFile(str(ROOT / 'qml/World3D.qml')))
        self.assertFalse(c.errors(), c.errors())
        world = c.create()
        # Ego lane, adjacent lane, opposite lane (reversed direction), junction.
        lanes = [
            {'L': [[-10, -1.75], [60, -1.75]], 'R': [[-10, 1.75], [60, 1.75]],
             'lm': ['double', 'y'], 'rm': ['dash', 'w']},
            {'L': [[-10, 1.75], [60, 1.75]], 'R': [[-10, 5.25], [60, 5.25]],
             'lm': ['dash', 'w'], 'rm': ['solid', 'w']},
            {'L': [[60, -5.25], [-10, -5.25]], 'R': [[60, -1.75], [-10, -1.75]],
             'lm': ['solid', 'w'], 'rm': ['double', 'y']},
            {'L': [[60, -7], [80, -7]], 'R': [[60, 7], [80, 7]], 'j': True},
        ]
        mapped = scene.build_map_scene(lanes, [{'x': 20, 'y': 10, 'l': 3, 'w': 3, 'h': 3}])
        mapped['ver'] = 4
        world.setProperty('carlaMap', mapped)
        # Includes all seven requested cases, both boundaries and roll-only tilt.
        transitions = [(0, False), (.40, True), (.20, True), (.39, True),
                       (.14, False), (-.60, True), (0, False)]
        for pitch, slope in transitions:
            with self.subTest(pitch=pitch, slope=slope):
                pkt = packet()
                pkt['ego'].update(pitch=0.1, roll=9)
                pkt['lane']['road_grade_deg'] = pitch
                pkt['road_grade_deg'] = pitch
                pkt.update(map_ver=4, map_xf={'x': 0, 'y': 0, 'rot': 0})
                world.setProperty('sc', build(pkt))
                self.app.processEvents()
                if not slope:
                    QTest.qWait(550)
                self.assertEqual(world.property('slopeMode'), slope)
                self.assertTrue(world.property('mapOn'))
                self.assertEqual(world.property('mapBoxesOn'), not slope)
                self.assertEqual(world.property('mapRoadOn'), not slope)
                self.assertEqual(world.property('mapLaneOn'), not slope)
                self.assertEqual(world.property('sceneRoadOn'), slope)
                self.assertEqual(world.findChild(QObject, 'sceneRoad3D').property('visible'), slope)
                # Actual mesh layers retain every map band/marking and only change visibility.
                geometries = world.findChildren(RibbonGeometry)
                bands = [g for g in geometries if g.bands]
                self.assertEqual(len(bands), 1)
                self.assertEqual(len(bands[0].bands), 4)
                self.assertEqual(bands[0].bands[2], mapped['bands'][2])  # opposite lane
                self.assertEqual(bands[0].bands[3], mapped['bands'][3])  # intersection
                self.assertEqual(bands[0].parent().property('visible'), not slope)
                markings = [g for g in geometries if g.strips]
                self.assertGreaterEqual(len(markings), 3)  # solid/dashed white and yellow centerline
                self.assertTrue(all(g.parent().property('visible') == (not slope) for g in markings))
                overlay = world.property('lines')
                if hasattr(overlay, 'toVariant'):
                    overlay = overlay.toVariant()
                self.assertTrue(all(line['kind'] == 'ego' for line in overlay) if not slope else len(overlay) > 2)
                debug = QMetaObject.invokeMethod(world, 'debugLine', Qt.DirectConnection, Q_RETURN_ARG('QVariant'))
                for label, expected in (('slopeMode', slope), ('mapRoadVisible', not slope),
                                        ('mapLaneVisible', not slope), ('sceneRoad3DVisible', slope)):
                    self.assertIn(label + '=' + str(int(expected)), debug)
                self.assertEqual(world.property('position'), QVector3D(0, 0, 0))
                self.assertEqual(world.property('eulerRotation'), QVector3D(0, 0, 0))
        # No ready/matching map: the scene road remains available, even in flat mode.
        world.setProperty('carlaMap', {'ver': 99})
        self.assertFalse(world.property('mapRoadOn'))
        self.assertTrue(world.property('sceneRoadOn'))
        world.deleteLater()

    def test_road_grade_drives_mode_independently_of_vehicle_pitch(self):
        from PySide6.QtCore import QUrl, QMetaObject, Qt, Q_RETURN_ARG
        from PySide6.QtQml import QQmlEngine, QQmlComponent
        engine = QQmlEngine()
        c = QQmlComponent(engine, QUrl.fromLocalFile(str(ROOT / 'qml/World3D.qml')))
        world = c.create()
        world.setProperty('carlaMap', {'ver': 4})
        for body, grade, expected in ((.1, 0, False), (.1, 6, True), (0, -6, True), (6, 0, False),
                                      (.1, .6, True), (.1, .2, True), (.1, .14, False),
                                      (6, .2, False)):
            with self.subTest(body=body, grade=grade):
                # Generate actual grade field in bridge; preserve all local geometry.
                pkt = packet(math.tan(math.radians(grade)), grade, ego_pitch=body)
                pkt.update(map_ver=4, map_xf={'x': 0, 'y': 0, 'rot': 0})
                self.assertEqual(pkt['ego']['pitch'], body)
                self.assertAlmostEqual(pkt['lane']['road_grade_deg'], grade, places=3)
                self.assertIn('road_grade_deg', pkt)
                decoded = json.loads(json.dumps(pkt))
                self.assertIn('road_grade_deg', decoded)
                self.assertEqual(decoded['road_grade_deg'], round(grade, 2))
                result = build(decoded)
                self.assertEqual(result['road_grade_deg'], decoded['road_grade_deg'])
                world.setProperty('sc', result)
                self.app.processEvents()
                if not expected:
                    QTest.qWait(550)
                self.assertEqual(world.property('slopeMode'), expected)
                self.assertEqual(world.property('mapRoadOn'), not expected)
                self.assertEqual(world.property('mapLaneOn'), not expected)
                self.assertEqual(world.property('sceneRoadOn'), expected)
                self.assertEqual(world.property('mapBoxesOn'), not expected)
                debug = QMetaObject.invokeMethod(world, 'debugLine', Qt.DirectConnection, Q_RETURN_ARG('QVariant'))
                self.assertIn('egoPitch=' + str(body), debug)
                self.assertIn('roadGrade=' + str(grade), debug)
                with patch('builtins.print') as log:
                    bridge.debug_scene_packet(pkt)
                line = log.call_args.args[0]
                self.assertIn('egoPitch=' + str(body), line)
                self.assertIn('roadGrade=' + str(grade), line)
        # Old/missing road-grade packet must never switch based on body pitch.
        pkt = packet(ego_pitch=6)
        del pkt['lane']['road_grade_deg']
        del pkt['road_grade_deg']
        world.setProperty('sc', build(pkt))
        self.app.processEvents()
        self.assertFalse(world.property('slopeMode'))
        world.deleteLater()

    def test_entire_hill_plateau_descent_original_height_scenario(self):
        from PySide6.QtCore import QObject, QUrl, QMetaObject, Qt, Q_RETURN_ARG
        from PySide6.QtGui import QVector3D
        from PySide6.QtQml import QQmlEngine, QQmlComponent
        engine = QQmlEngine()
        c = QQmlComponent(engine, QUrl.fromLocalFile(str(ROOT / 'qml/World3D.qml')))
        self.assertFalse(c.errors(), c.errors())
        world = c.create()
        world.setProperty('carlaMap', {'ver': 1})
        def feed(z, grade):
            pkt = packet()
            pkt['ego']['z'] = z
            pkt['road_grade_deg'] = grade
            pkt.update(map_ver=1, map_xf={'x': 0, 'y': 0, 'rot': 0})
            sc = build(json.loads(json.dumps(pkt)))
            self.assertEqual(sc['ego']['z'], z)
            world.setProperty('sc', sc)
            self.app.processEvents()
        def check(mode):
            road = mode == 'ROAD3D_MODE'
            self.assertEqual(world.property('renderMode'), mode)
            self.assertEqual(world.property('slopeMode'), road)
            for name in ('mapRoadOn', 'mapLaneOn', 'mapBoxesOn'):
                self.assertEqual(world.property(name), not road)
            self.assertEqual(world.property('sceneRoadOn'), road)
            self.assertEqual(world.property('position'), QVector3D(0, 0, 0))
            self.assertEqual(world.findChild(QObject, 'ego3d').property('position'), QVector3D(0, 0, 0))
        # All eight user-requested stages, in a single persistent World3D.
        feed(1, 0); check('MAP_MODE')
        self.assertIsNone(world.property('baseFlatZ'))
        for z, grade in ((1, .6), (2, .6), (3, 0), (3, .05), (2, -.6), (1.5, 0)):
            feed(z, grade)
            check('ROAD3D_MODE')
            self.assertEqual(world.property('baseFlatZ'), 1)
            self.assertFalse(world.findChild(QObject, 'mapReturnTimer').property('running'))
            if z == 3 or z == 1.5:
                QTest.qWait(550)
                check('ROAD3D_MODE')  # Flat plateau must never return, even after 500ms.
        feed(1.05, .05)
        QTest.qWait(250)
        check('ROAD3D_MODE')
        feed(1.05, .05)  # Frequent valid packets must not restart the dwell timer.
        QTest.qWait(300)
        check('MAP_MODE')
        self.assertEqual(world.property('baseFlatZ'), 1)
        debug = QMetaObject.invokeMethod(world, 'debugLine', Qt.DirectConnection, Q_RETURN_ARG('QVariant'))
        for value in ('mode=MAP', 'roadGrade=0.05', 'egoZ=1.05', 'baseFlatZ=1.00', 'heightDelta=0.05'):
            self.assertIn(value, debug)
        world.deleteLater()

    def test_map_return_requires_continuous_grade_height_and_valid_telemetry(self):
        from PySide6.QtCore import QObject, QUrl
        from PySide6.QtQml import QQmlEngine, QQmlComponent
        engine = QQmlEngine()
        c = QQmlComponent(engine, QUrl.fromLocalFile(str(ROOT / 'qml/World3D.qml')))
        world = c.create()
        def feed(z, grade):
            world.setProperty('sc', {'source': 'CARLA', 'ego': {'z': z}, 'road_grade_deg': grade})
            self.app.processEvents()
        feed(1, .4)
        self.assertEqual(world.property('baseFlatZ'), 1)
        timer = world.findChild(QObject, 'mapReturnTimer')
        self.assertEqual(timer.property('interval'), 500)
        for bad_z, bad_grade in ((1.21, 0), (1, .16), (None, 0), (1, None)):
            feed(1.05, .05)
            QTest.qWait(250)
            self.assertTrue(world.property('slopeMode'))
            feed(bad_z, bad_grade)
            self.assertFalse(timer.property('running'))
            feed(1.05, .05)
            QTest.qWait(300)  # Old partial dwell must not count toward this one.
            self.assertTrue(world.property('slopeMode'))
            feed(2, 0)
            self.assertFalse(timer.property('running'))
            self.assertEqual(world.property('baseFlatZ'), 1)
        feed(1.05, .05)
        QTest.qWait(550)
        self.assertFalse(world.property('slopeMode'))
        # A new hill captures its own entry height; only now may baseFlatZ change.
        feed(2, -.6)
        self.assertTrue(world.property('slopeMode'))
        self.assertEqual(world.property('baseFlatZ'), 2)
        feed(3, 0)
        self.assertEqual(world.property('baseFlatZ'), 2)
        self.assertTrue(world.property('slopeMode'))
        world.deleteLater()

    def test_backend_slope_and_ui_packets_interleaved(self):
        from hmi.backend import HmiBackend
        with patch('hmi.backend.AlertSound') as sound:
            sound.return_value.muted = True
            sound.return_value.backend = 'mock'
            backend = HmiBackend(sound=False)
        backend._timer.stop()
        backend._diag_timer.stop()
        with patch('socket.socket', side_effect=AssertionError('network forbidden')):
            for order in (('scene', 'ui'), ('ui', 'scene')):
                pkt = packet(.1, 5)
                ui = {'type': 'carla_ui_state', 'night_mode': True,
                      'lane_assist_enabled': True, 'lane_assist_active': True}
                for name in order:
                    backend.sink('carla', pkt if name == 'scene' else ui, 100)
                with patch('hmi.backend.time.monotonic', return_value=100):
                    backend._tick()
                self.assertTrue(backend._state['night_mode'])
                self.assertTrue(backend._state['lane_assist_active'])
                self.assertEqual(backend._state['scene']['vehicles'][0]['local_z'], 0)
                self.assertEqual(backend._carla_store['data'], pkt)
        backend.deleteLater()


if __name__ == '__main__':
    unittest.main()
