#!/usr/bin/env python3
"""The X30 URDF: the official model's legs and LiDARs, base_link, imu_link, the driver's joints."""

import os
import sys
import unittest
from xml.etree import ElementTree

HERE = os.path.dirname(os.path.abspath(__file__))
PACKAGE = os.path.dirname(HERE)
XACRO = os.path.join(PACKAGE, 'urdf', 'x30.urdf.xacro')
# x30_driver from the workspace, or from the source tree next to this package
sys.path.append(os.path.join(PACKAGE, '..', 'x30_driver', 'src'))

try:
    import xacro
except ImportError:  # running outside ROS without the xacro module
    xacro = None


def urdf():
    """Return the expanded model."""
    return ElementTree.fromstring(xacro.process_file(XACRO).toxml())


def joint(root, name):
    found = root.find("joint[@name='{}']".format(name))
    assert found is not None, 'no joint ' + name
    return found


def numbers(element, tag='origin', attribute='xyz'):
    """Return the numbers of an attribute of a child element."""
    return tuple(float(v) for v in element.find(tag).get(attribute).split())


def parent_of(root, child):
    for j in root.findall('joint'):
        if j.find('child').get('link') == child:
            return j.find('parent').get('link')
    return ''


@unittest.skipIf(xacro is None, 'xacro is not installed')
class TestUrdf(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.root = urdf()

    def test_joint_names_are_the_drivers(self):
        """The 12 revolute joints are the names x30_driver publishes, in any order."""
        from x30_driver.protocol import JOINT_NAMES
        revolute = [j.get('name') for j in self.root.findall("joint[@type='revolute']")]
        self.assertEqual(sorted(revolute), sorted(JOINT_NAMES))

    def test_legs_are_the_official_model(self):
        """Positions, axes and limits as in X30.urdf."""
        r = self.root
        self.assertEqual(numbers(joint(r, 'FL_HipX_joint')), (0.291, 0.08, 0.0))
        self.assertEqual(numbers(joint(r, 'HR_HipX_joint')), (-0.291, -0.08, 0.0))
        self.assertEqual(numbers(joint(r, 'FR_HipY_joint')), (0.0, -0.11675, 0.0))
        self.assertEqual(numbers(joint(r, 'HL_HipY_joint')), (0.0, 0.11675, 0.0))
        self.assertEqual(numbers(joint(r, 'HL_Knee_joint')), (0.0, 0.0, -0.3))
        self.assertEqual(numbers(joint(r, 'FL_Ankle')), (0.0, 0.0, -0.31))
        self.assertEqual(numbers(joint(r, 'FR_HipX_joint'), 'axis'), (-1.0, 0.0, 0.0))
        self.assertEqual(numbers(joint(r, 'FR_Knee_joint'), 'axis'), (0.0, -1.0, 0.0))
        limit = joint(r, 'FL_HipX_joint').find('limit')
        self.assertEqual((float(limit.get('lower')), float(limit.get('upper'))), (-0.323, 0.585))
        limit = joint(r, 'HR_HipX_joint').find('limit')
        self.assertEqual((float(limit.get('lower')), float(limit.get('upper'))), (-0.585, 0.323))
        limit = joint(r, 'HL_Knee_joint').find('limit')
        self.assertEqual([float(limit.get(k)) for k in ('lower', 'upper', 'effort', 'velocity')],
                         [0.349, 2.531, 150.0, 16.1])

    def test_inertials_and_mirrored_meshes(self):
        """Per-leg inertials and mesh scales come through the macro unchanged."""
        r = self.root
        hip = r.find("link[@name='HR_HIP']")
        self.assertEqual(numbers(hip.find('inertial'), 'origin', 'rpy'), (3.14159, 3.14159, 0.0))
        self.assertEqual(numbers(hip.find('visual/geometry'), 'mesh', 'scale'), (-1.0, -1.0, 1.0))
        thigh = r.find("link[@name='FR_THIGH']")
        self.assertEqual(numbers(thigh.find('inertial')), (-0.0087433, 0.023551, -0.055154))
        self.assertEqual(numbers(thigh.find('visual/geometry'), 'mesh', 'scale'), (1.0, -1.0, 1.0))
        torso = r.find("link[@name='TORSO']/inertial/mass")
        self.assertEqual(float(torso.get('value')), 30.7)
        masses = [float(m.get('value')) for m in r.iter('mass')]
        self.assertAlmostEqual(sum(masses), 30.7 + 4 * (1.43 + 4.0809 + 0.71386 + 0.06) + 4e-5)

    def test_lidar_and_imu_frames(self):
        """The four LiDARs of the official model on TORSO; imu_link at base_link."""
        r = self.root
        self.assertEqual(numbers(joint(r, 'LIDAR_FRONT_UP_joint')), (0.44367, 0.0, 0.05415))
        self.assertEqual(numbers(joint(r, 'LIDAR_FRONT_DOWN_joint'), 'origin', 'rpy'),
                         (0.0, 0.4363, 0.0))
        self.assertEqual(numbers(joint(r, 'LIDAR_REAR_UP_joint')), (-0.46576, 0.0, 0.03484))
        self.assertEqual(numbers(joint(r, 'LIDAR_REAR_DOWN_joint'), 'origin', 'rpy'),
                         (0.0, -0.8029, 0.0))
        for name in ('LIDAR_FRONT_UP', 'LIDAR_FRONT_DOWN', 'LIDAR_REAR_UP', 'LIDAR_REAR_DOWN'):
            self.assertEqual(parent_of(r, name), 'TORSO')
        self.assertEqual(numbers(joint(r, 'imu_joint')), (0.0, 0.0, 0.0))
        self.assertEqual(parent_of(r, 'imu_link'), 'base_link')

    def test_meshes_are_in_the_package(self):
        files = {m.get('filename') for m in self.root.iter('mesh')}
        self.assertEqual(len(files), 4)
        for name in files:
            self.assertTrue(name.startswith('package://x30_description/meshes/'), name)
            path = os.path.join(PACKAGE, 'meshes', name.rsplit('/', 1)[1])
            self.assertTrue(os.path.isfile(path), name)

    def test_tree_rooted_at_base_link(self):
        """One tree from base_link (TORSO under it); no frame owned by odometry or navigation."""
        r = self.root
        names = {link.get('name') for link in r.findall('link')}
        children = [j.find('child').get('link') for j in r.findall('joint')]
        self.assertEqual(len(children), len(set(children)))
        self.assertEqual(names - set(children), {'base_link'})
        self.assertEqual(parent_of(r, 'TORSO'), 'base_link')
        self.assertEqual(len(names), 23)
        self.assertFalse(names & {'body', 'odom', 'map', 'camera_init'})


if __name__ == '__main__':
    unittest.main()
