"""Package conventions checked by every package's test_style.py."""

from __future__ import annotations

import ast
from pathlib import Path

from x30_utils.testing.py38 import py38_problems

MAX_LINES = 300
MAX_TEST_LINES = 400
ROS_MODULES = (
    'rospy', 'roslib', 'rosgraph', 'rospkg', 'roslaunch', 'tf', 'tf2_ros', 'tf2_py',
    'message_filters', 'actionlib', 'cv_bridge', 'dynamic_reconfigure',
)  # fmt: skip
ROS_SUFFIXES = ('.msg', '.srv')
NODE_BASES = {'X30Node'}


def check_package(root: Path, py38: bool | None = None) -> list[str]:
    """Return every convention problem under root (Python 3.8 checks when ruff targets py38)."""
    root = Path(root)
    if py38 is None:
        py38 = targets_py38(root)
    problems = []
    for path in sorted(root.rglob('*.py')):
        rel = path.relative_to(root)
        if rel.parts[0] in ('build', 'install', 'log') or 'resource' in rel.parts:
            continue
        problems += _check_file(path, rel, py38)
    return problems


def targets_py38(root: Path) -> bool:
    """Return True when the nearest ruff.toml above root targets Python 3.8."""
    for directory in (root, *root.resolve().parents):
        config = directory / 'ruff.toml'
        if config.is_file():
            return 'target-version = "py38"' in config.read_text(encoding='utf-8')
    return False


def _check_file(path: Path, rel: Path, py38: bool) -> list[str]:
    text = path.read_text(encoding='utf-8')
    is_test = rel.parts[0] == 'test'
    limit = MAX_TEST_LINES if is_test else MAX_LINES
    problems = []
    lines = text.count('\n') + 1
    if lines > limit:
        problems.append(f'{rel}: {lines} lines (max {limit}), split it')
    tree = ast.parse(text, filename=str(path))
    if 'core' in rel.parts[:-1] and not is_test:
        problems += [f'{rel}:{line} core/ imports {name}' for name, line in _ros_imports(tree)]
    problems += _shadowed_members(tree, rel)
    problems += _long_docstrings(tree, rel)
    if py38:
        problems += py38_problems(text, tree, rel)
    return problems


def _long_docstrings(tree: ast.AST, rel: Path) -> list[str]:
    """Return the docstrings spanning more than one line (the convention is one line)."""
    kinds = (ast.Module, ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef)
    problems = []
    for node in ast.walk(tree):
        doc = ast.get_docstring(node, clean=False) if isinstance(node, kinds) else None
        if doc and '\n' in doc.strip():
            line = node.body[0].lineno
            problems.append(f'{rel}:{line} docstring of more than one line, shorten it')
    return problems


def _ros_imports(tree: ast.AST):
    for node in ast.walk(tree):
        names = []
        if isinstance(node, ast.Import):
            names = [a.name for a in node.names]
        elif isinstance(node, ast.ImportFrom) and node.module:
            names = [node.module]
        for name in names:
            if name.split('.')[0] in ROS_MODULES or name.endswith(ROS_SUFFIXES):
                yield name, node.lineno
            if name.startswith(('x30_utils.ros', 'x30_utils.testing')):
                yield name, node.lineno


def _node_members() -> set:
    """Return what X30Node defines: a subclass must not replace any of it."""
    try:
        from x30_utils.ros.node import NODE_INTERNALS, X30Node
    except ImportError:  # no rospy: the check needs the base class
        return set()
    return set(NODE_INTERNALS) | {n for n in dir(X30Node) if not n.startswith('__')}


def _shadowed_members(tree: ast.AST, rel: Path) -> list[str]:
    forbidden = _node_members()
    problems = []
    for cls in (n for n in ast.walk(tree) if isinstance(n, ast.ClassDef)):
        bases = {
            b.attr if isinstance(b, ast.Attribute) else getattr(b, 'id', '') for b in cls.bases
        }
        if not bases & NODE_BASES:
            continue
        for name, line in _defined_names(cls):
            if name in forbidden:
                problems.append(f'{rel}:{line} {cls.name}.{name} shadows X30Node')
    return problems


def _defined_names(cls: ast.ClassDef):
    for item in cls.body:
        if isinstance(item, (ast.FunctionDef, ast.AsyncFunctionDef)):
            yield item.name, item.lineno
    for node in ast.walk(cls):
        targets = node.targets if isinstance(node, ast.Assign) else []
        if isinstance(node, (ast.AnnAssign, ast.AugAssign)):
            targets = [node.target]
        for target in targets:
            for t in ast.walk(target):
                if isinstance(t, ast.Attribute) and getattr(t.value, 'id', '') == 'self':
                    yield t.attr, node.lineno
