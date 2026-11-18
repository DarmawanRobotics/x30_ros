"""tools/: Deep Robotics' autonomy off and on, the offline dependency check, the source archive."""

from __future__ import annotations

import os
from pathlib import Path
import re
import shutil
import subprocess
import tarfile
import time
import xml.etree.ElementTree as ET

import pytest

ROOT = Path(__file__).resolve().parents[2]
TOOLS = ROOT / 'tools'
SOURCES = ('fast_lio', 'livox_ros_driver', 'hikmicro_msgs', 'hikmicro_driver', 'hertzino_msgs',
           'hertzino_driver')  # fmt: skip


def run(script: str, *args: str, env: dict | None = None, cwd=None, timeout: float = 60.0):
    command = ['bash', str(TOOLS / script), *args]
    environment = {**os.environ, **(env or {})}
    return subprocess.run(command, capture_output=True, text=True, env=environment, cwd=cwd,
                          timeout=timeout, check=False)  # fmt: skip


def wait_until(predicate, timeout: float = 5.0) -> bool:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(0.05)
    return predicate()


def listed(pid: int) -> bool:
    out = subprocess.run(['ps', '-o', 'args=', '-p', str(pid)], capture_output=True, text=True,
                         check=False)  # fmt: skip
    return 'jy_cog' in out.stdout


# ---------------------------------------------------------------- dr_services.sh


@pytest.fixture
def jy(tmp_path):
    """A navigation host's jy_cog with its kill scripts, and processes started from it."""
    root = tmp_path / 'jy_cog'
    (root / 'system' / 'conf').mkdir(parents=True)
    (root / 'system' / 'scripts').mkdir()
    (root / 'system' / 'conf' / 'autorun_slam').write_text('true\n')
    log = tmp_path / 'scripts.log'
    log.touch()
    for name in ('kill_nav.sh', 'kill_mapping.sh'):
        (root / 'system' / 'scripts' / name).write_text(f'echo {name} >> {log}\n')
    processes = []

    def spawn(path: str, *args: str) -> subprocess.Popen:
        """Run path of jy_cog; with args as a script a shell runs ('bash <path> <args>')."""
        if args:
            line = 'exec -a bash python3 -c "import time; time.sleep(60)" "$@"'
            command = ['bash', '-c', line, '_', f'{root}/{path}', *args]
        else:
            command = ['bash', '-c', f'exec -a "{root}/{path}" sleep 60']
        process = subprocess.Popen(command, stderr=subprocess.DEVNULL)
        processes.append(process)
        assert wait_until(lambda: listed(process.pid))
        return process

    yield root, log, spawn
    for process in processes:
        process.kill()
        process.wait()


def services(root: Path, *args: str):
    return run('dr_services.sh', *args, env={'JY_COG': str(root)})


def test_status_sorts_the_jy_cog_processes(jy):
    root, _, spawn = jy
    nav, driver = spawn('nav/bin/move_base'), spawn('drivers/livox_lidar_publisher2')
    out = services(root, 'status')
    assert out.returncode == 0, out.stdout + out.stderr
    autonomy, kept = out.stdout.split('kept (')
    assert 'autorun_slam: true' in autonomy and f'{nav.pid} ' in autonomy
    assert f'{driver.pid} ' in kept.split('other jy_cog')[0]


def test_the_program_decides_never_its_arguments(jy):
    """A wrapper's arguments (livox) never keep it; planner, URDF and bridge are autonomy."""
    root, _, spawn = jy
    wrapper = spawn('localization/scripts/localization.sh', 'false', 'livox')
    lidar = spawn('drivers/scripts/lidar_driver.sh', 'livox')
    imu = spawn('drivers/lib/yesense_imu/yesense_imu_node')
    planner = spawn('planner/lib/local_planner/localPlanner')
    plan = spawn('nav/scripts/plan.sh', 'false')
    urdf = spawn('drivers/scripts/urdf.sh')
    bridge = spawn('transfer/lib/message_transformer_cpp/udp_receiver')
    monitor = spawn('cpu_monitor/lib/cpu_monitor/cpu_monitor_node')
    out = services(root, 'status')
    assert out.returncode == 0, out.stdout + out.stderr
    autonomy, rest = out.stdout.split('kept (')
    kept, other = rest.split('other jy_cog')
    for process in (wrapper, planner, plan, urdf, bridge):
        assert f'{process.pid} ' in autonomy, out.stdout
    assert f'{lidar.pid} ' in kept and f'{imu.pid} ' in kept, out.stdout
    assert f'{monitor.pid} ' in other.split('load ')[0], out.stdout
    assert re.search(r"udp 43897 \(x30_driver's reports\): (free|in use|unknown)", out.stdout)


def test_off_stops_the_autonomy_now_and_at_boot_and_keeps_the_drivers(jy):
    root, log, spawn = jy
    nav, mapping = spawn('nav/bin/move_base'), spawn('slam/bin/mapping_node')
    wrapper, planner = spawn('nav/scripts/plan.sh', 'false'), spawn('planner/lib/x/localPlanner')
    driver = spawn('drivers/livox_lidar_publisher2')
    bridge = spawn('transfer/lib/message_transformer_cpp/udp_sender')
    monitor = spawn('cpu_monitor/lib/cpu_monitor/cpu_monitor_node')
    began = time.monotonic()
    out = services(root, 'off')
    assert out.returncode == 0, out.stdout + out.stderr
    assert time.monotonic() - began < 5.0  # all at once, each answering SIGINT
    conf = root / 'system' / 'conf'
    assert (conf / 'autorun_slam').read_text() == 'false\n'  # the file's own format kept
    assert (conf / 'autorun_slam.x30_backup').read_text() == 'true\n'
    assert log.read_text().split() == ['kill_nav.sh', 'kill_mapping.sh']  # Deep Robotics' own
    stopped = (nav, mapping, wrapper, planner, bridge)
    assert wait_until(lambda: all(p.poll() is not None for p in stopped))
    assert driver.poll() is None and monitor.poll() is None
    assert 'still running:\n  none' in out.stdout


def children(pid: int) -> list[int]:
    out = subprocess.run(['ps', '-o', 'pid=', '--ppid', str(pid)], capture_output=True,
                         text=True, check=False)  # fmt: skip
    return [int(p) for p in out.stdout.split()]


def script(root: Path, path: str, body: str) -> subprocess.Popen:
    """Run a script of jy_cog that runs body in the foreground, as a wrapper does."""
    file = root / path
    file.parent.mkdir(parents=True, exist_ok=True)
    file.write_text(f'{body}\n')
    process = subprocess.Popen(['bash', str(file)], stderr=subprocess.DEVNULL)
    assert wait_until(lambda: children(process.pid))
    return process


def test_off_stops_what_a_wrapper_started_and_leaves_what_it_shares_with_kept(jy):
    """urdf.sh's robot_state_publisher goes with it; a script that runs a driver too stays."""
    root, _, _ = jy
    urdf = script(root, 'drivers/scripts/urdf.sh', 'sleep 60')  # a node from outside jy_cog
    node = children(urdf.pid)[0]
    lidar_node = f"bash -c 'exec -a {root}/drivers/lidar_node sleep 60'"  # a kept child
    shared = script(root, 'nav/scripts/bringup.sh', lidar_node)
    lidar = children(shared.pid)[0]
    try:
        status = services(root, 'status').stdout.split('kept (')
        assert f'{node} ' in status[0] and f'{lidar} ' in status[1]
        out = services(root, 'off')
        assert out.returncode == 0, out.stdout + out.stderr
        assert f'left {shared.pid} ' in out.stdout, out.stdout
        assert wait_until(lambda: urdf.poll() is not None and not Path(f'/proc/{node}').exists())
        assert shared.poll() is None and Path(f'/proc/{lidar}').exists()
    finally:
        for pid in (node, lidar):
            subprocess.run(['kill', str(pid)], check=False)
        for process in (urdf, shared):
            process.kill()
            process.wait()


def test_on_restores_what_off_changed(jy):
    root, _, _ = jy
    assert services(root, 'off').returncode == 0
    out = services(root, 'on')
    assert out.returncode == 0 and 'autorun_slam true again' in out.stdout
    conf = root / 'system' / 'conf'
    assert (conf / 'autorun_slam').read_text() == 'true\n'
    assert not (conf / 'autorun_slam.x30_backup').exists()
    assert 'no backup from off' in services(root, 'on').stdout  # once only


def test_off_twice_keeps_the_first_backup(jy):
    root, _, _ = jy
    services(root, 'off')
    out = services(root, 'off')
    assert 'autorun_slam is false already' in out.stdout
    assert (root / 'system' / 'conf' / 'autorun_slam.x30_backup').read_text() == 'true\n'


def test_dry_run_changes_nothing(jy):
    root, log, spawn = jy
    nav = spawn('nav/bin/move_base')
    out = services(root, 'off', '--dry-run')
    assert out.returncode == 0 and 'would run: bash' in out.stdout
    assert (root / 'system' / 'conf' / 'autorun_slam').read_text() == 'true\n'
    assert not (root / 'system' / 'conf' / 'autorun_slam.x30_backup').exists()
    assert log.read_text() == '' and nav.poll() is None


def test_elsewhere_or_unknown_commands_are_refused(tmp_path):
    out = services(tmp_path / 'nowhere', 'status')
    assert out.returncode == 1 and 'run this on the navigation host' in out.stdout
    assert services(tmp_path, 'restart').returncode == 2


# ---------------------------------------------------------------- check_deps.sh


def array(script: Path, name: str) -> list[str]:
    """Return the words of a bash array of a script (comments dropped)."""
    body = re.search(rf'^{name}=\((.*?)\)', script.read_text(), re.S | re.M)
    assert body, name
    return [w for line in body.group(1).splitlines() for w in line.split('#')[0].split()]


@pytest.fixture
def system(tmp_path):
    """A workspace, ROS, the Livox SDK and a dpkg-query answering for a package list."""
    src = tmp_path / 'ws' / 'src'
    for name in SOURCES:
        (src / name).mkdir(parents=True)
        (src / name / 'package.xml').write_text(f'<package><name>{name}</name></package>')
    (src / 'fast_lio' / 'include' / 'ikd-Tree').mkdir(parents=True)
    (src / 'fast_lio' / 'include' / 'ikd-Tree' / 'ikd_Tree.cpp').touch()
    ros = tmp_path / 'ros'
    ros.mkdir()
    (ros / 'setup.bash').touch()
    sdk = tmp_path / 'liblivox_sdk_static.a'
    sdk.touch()
    bin_dir = tmp_path / 'bin'
    bin_dir.mkdir()
    fake = bin_dir / 'dpkg-query'
    fake.write_text(
        '#!/bin/bash\nshift 2  # -W -f=...\n'
        'for p; do grep -qxF "$p" "$INSTALLED" && echo "$p ii "; done\nexit 0\n'
    )
    fake.chmod(0o755)
    installed = tmp_path / 'installed.txt'
    packages = array(TOOLS / 'check_deps.sh', 'BUILD')
    installed.write_text('\n'.join(packages) + '\n')
    env = {
        'PATH': f'{bin_dir}:{os.environ["PATH"]}',
        'INSTALLED': str(installed),
        'ROS_DIR': str(ros),
        'LIVOX_SDK_LIB': str(sdk),
        'HOME': str(tmp_path),  # no rosdep cache
    }
    return src, env, installed, sdk


def test_check_deps_passes_when_everything_is_there(system):
    src, env, _, _ = system
    out = run('check_deps.sh', '--src', str(src), env=env)
    assert out.returncode == 0, out.stdout
    assert 'everything there' in out.stdout and 'MISS' not in out.stdout


def test_check_deps_names_what_is_missing_and_how(system):
    src, env, installed, sdk = system
    installed.write_text(installed.read_text().replace('ros-noetic-pcl-ros\n', ''))
    sdk.unlink()
    (src / 'fast_lio' / 'include' / 'ikd-Tree' / 'ikd_Tree.cpp').unlink()
    shutil.copytree(src / 'hikmicro_msgs', src / 'drivers' / 'hikmicro_msgs')
    out = run('check_deps.sh', '--src', str(src), env=env)
    assert out.returncode == 1
    assert 'apt      MISS  ros-noetic-pcl-ros' in out.stdout
    assert 'sudo apt install -y ros-noetic-pcl-ros' in out.stdout
    assert re.search(r'DUP\s+hikmicro_msgs in src/.*src/', out.stdout), out.stdout
    assert 'ikd-Tree submodule' in out.stdout and 'livox    MISS' in out.stdout


def test_check_deps_builds_past_a_user_site_that_breaks_setuptools(system, tmp_path):
    """A pip --user setuptools that fails to import (catkin's error) asks for PYTHONNOUSERSITE."""
    src, env, _, _ = system
    env = {**env, 'PYTHONUSERBASE': str(tmp_path / 'user'), 'PYTHONNOUSERSITE': ''}
    query = 'import site; print(site.getusersitepackages(), site.ENABLE_USER_SITE)'
    answer = subprocess.run(
        ['python3', '-c', query],
        env={**os.environ, **env},
        capture_output=True,
        text=True,
        check=True,
    )
    site, enabled = answer.stdout.split()
    if enabled != 'True':
        pytest.skip('this python has no user site (a virtual environment)')
    (Path(site) / 'setuptools').mkdir(parents=True)
    message = "module 'importlib_metadata' has no attribute 'EntryPoints'"
    (Path(site) / 'setuptools' / '__init__.py').write_text(f'raise AttributeError({message!r})\n')
    out = run('check_deps.sh', '--src', str(src), env=env)
    assert out.returncode == 0, out.stdout
    assert re.search(r'python\s+WARN\s+setuptools in .*user.* breaks catkin', out.stdout)
    assert 'everything there: PYTHONNOUSERSITE=1 catkin_make' in out.stdout


def test_check_deps_for_the_perception_host_checks_its_camera_drivers_only(system):
    src, env, installed, sdk = system
    sdk.unlink()  # no FAST-LIO there: neither the SDK nor its sources
    shutil.rmtree(src / 'fast_lio')
    shutil.rmtree(src / 'livox_ros_driver')
    out = run('check_deps.sh', '--perception', '--src', str(src), env=env)
    assert out.returncode == 0, out.stdout
    assert 'of the perception host' in out.stdout and 'livox' not in out.stdout
    assert 'neupan' not in out.stdout
    whitelist = 'x30_utils;x30_bringup;hikmicro_msgs;hikmicro_driver;hertzino_msgs;hertzino_driver'
    assert f"-DCATKIN_WHITELIST_PACKAGES='{whitelist}'" in out.stdout
    script = TOOLS / 'check_deps.sh'
    assert set(array(script, 'PERCEPTION_BUILD')) <= set(array(script, 'BUILD'))
    installed.write_text(installed.read_text().replace('libtinyxml2-dev\n', ''))
    missing = run('check_deps.sh', '--perception', '--src', str(src), env=env)
    assert missing.returncode == 1 and 'sudo apt install -y libtinyxml2-dev' in missing.stdout


def test_check_deps_tells_whether_neupan_is_there(system):
    src, env, _, _ = system
    out = run('check_deps.sh', '--src', str(src), env=env)
    assert re.search(r'neupan\s+--\s+no .*/neupan: for controller neupan', out.stdout)
    assert out.returncode == 0  # optional


def test_check_deps_adds_the_test_packages_when_asked(system):
    src, env, _, _ = system
    out = run('check_deps.sh', '--tests', '--src', str(src), env=env)
    assert out.returncode == 1 and 'apt      MISS  python3-pytest' in out.stdout


def test_the_apt_lists_cover_every_dependency_of_the_stack():
    script = TOOLS / 'check_deps.sh'
    listed_packages = {p for name in ('BUILD', 'TESTS', 'GUI') for p in array(script, name)}
    ours = {p.parent.name for p in ROOT.glob('*/package.xml')}
    keys = set()
    for manifest in ROOT.glob('*/package.xml'):
        for element in ET.parse(manifest).getroot():
            if element.tag.endswith('depend'):
                keys.add(element.text.strip())
    needed = set()
    for key in keys - ours - set(SOURCES):
        if key.startswith('python3'):
            needed.add('python3-dev' if key == 'python3' else key)
        else:
            needed.add('ros-noetic-' + key.replace('_', '-'))
    assert needed - listed_packages == set()


# ---------------------------------------------------------------- pack_sources.sh


def git(*args: str, cwd: Path):
    subprocess.run(['git', *args], cwd=cwd, check=True, capture_output=True,
                   env={**os.environ, 'GIT_AUTHOR_NAME': 't', 'GIT_AUTHOR_EMAIL': 't@t',
                        'GIT_COMMITTER_NAME': 't', 'GIT_COMMITTER_EMAIL': 't@t'})  # fmt: skip


def repository(path: Path, files: dict) -> str:
    path.mkdir(parents=True)
    for name, text in files.items():
        (path / name).parent.mkdir(parents=True, exist_ok=True)
        (path / name).write_text(text)
    git('init', '-q', cwd=path)
    git('checkout', '-q', '-b', 'main', cwd=path)
    git('add', '.', cwd=path)
    git('commit', '-q', '-m', 'init', cwd=path)
    return f'file://{path}'


@pytest.mark.skipif(shutil.which('git') is None, reason='git is needed to pack')
def test_pack_sources_brings_the_repositories_and_the_sdk_without_history(tmp_path):
    lio = repository(tmp_path / 'r' / 'lio', {'package.xml': '<name>fast_lio</name>'})
    cam = repository(tmp_path / 'r' / 'cam', {'package.xml': '<name>cam</name>'})
    sdk = repository(tmp_path / 'r' / 'sdk', {'sdk_core/include/livox_sdk.h': ''})
    repos = tmp_path / 'test.repos'
    repos.write_text(
        'repositories:\n'
        f'  third_party/FAST_LIO:\n    type: git\n    url: {lio}\n    version: main\n'
        f'  drivers/cam:\n    type: git\n    url: {cam}\n    version: main\n'
    )
    out = run('pack_sources.sh', 'out.tar.gz', '--skip', 'cam', cwd=tmp_path, timeout=120,
              env={'X30_REPOS': str(repos), 'LIVOX_SDK_URL': sdk})  # fmt: skip
    assert out.returncode == 0, out.stdout + out.stderr
    assert 'drivers/cam: skipped' in out.stdout
    with tarfile.open(tmp_path / 'out.tar.gz') as archive:
        names = archive.getnames()
        manifest = archive.extractfile('src/SOURCES.txt').read().decode()
    assert 'src/x30_ros/tools/pack_sources.sh' in names
    assert 'src/third_party/FAST_LIO/package.xml' in names
    assert 'Livox-SDK/sdk_core/include/livox_sdk.h' in names
    assert not any(n.startswith('src/drivers') for n in names)
    assert not any('.git' in Path(n).parts for n in names)
    assert re.search(rf'third_party/FAST_LIO {re.escape(lio)} main [0-9a-f]{{40}}', manifest)


def test_x30_repos_keeps_the_layout_the_packer_reads():
    entries = re.findall(r'^  (\S+):\n    type: git\n    url: (\S+)\n    version: (\S+)$',
                         (ROOT / 'x30.repos').read_text(), re.M)  # fmt: skip
    paths = [path for path, _, _ in entries]
    assert paths == ['third_party/FAST_LIO', 'third_party/livox_ros_driver',
                     'drivers/hikmicro_driver', 'drivers/hertzino_driver']  # fmt: skip
