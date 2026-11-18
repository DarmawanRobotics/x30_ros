"""Python 3.8 (Noetic) problems ruff cannot see; lines ending in '# py38: ok' are skipped."""

from __future__ import annotations

import ast

GENERICS = {'tuple', 'list', 'dict', 'set', 'frozenset', 'type'}
NEW_FUNCTIONS = {
    ('itertools', 'pairwise'): 'zip(xs, xs[1:])',
    ('functools', 'cache'): 'functools.lru_cache(maxsize=None)',
    ('math', 'lcm'): 'a plain helper',
    ('asyncio', 'to_thread'): 'loop.run_in_executor',
    ('typing', 'get_type_hints'): 'x30_utils.config.type_hints',
    ('importlib.resources', 'files'): 'rospkg.RosPack().get_path / os.path',
    ('ast', 'unparse'): 'source segments (ast.get_source_segment)',
    ('types', 'NoneType'): 'type(None)',
    ('argparse', 'BooleanOptionalAction'): "action='store_true'",
    ('math', 'nextafter'): 'an explicit tolerance',
}
NEW_MODULES = {
    'zoneinfo': 'datetime.timezone',
    'graphlib': 'a plain topological sort',
    'tomllib': 'a YAML or JSON file',
}
NEW_METHODS = {
    'removeprefix': 's[len(prefix):] if s.startswith(prefix) else s',
    'removesuffix': 's[: -len(suffix)] if suffix and s.endswith(suffix) else s',
    'bit_count': "bin(n).count('1')",
    'is_relative_to': 'x30_utils.io.resolve_inside or a try/relative_to',
    'with_stem': 'path.with_name(stem + path.suffix)',
}
NEW_KEYWORDS = {
    ('zip', 'strict'): 'zip() after an explicit length check',
    ('dataclass', 'slots'): 'a plain dataclass',
    ('dataclass', 'kw_only'): 'a plain dataclass',
    ('entry_points', 'group'): 'x30_utils.plugins.entry_points(group)',
}

PRAGMA = '# py38: ok'
SUFFIX = '(Python 3.8 / Noetic)'


def py38_problems(text: str, tree: ast.AST, rel) -> list[str]:
    """Return 'file:line message' for every construct that breaks on Python 3.8."""
    allowed = {n for n, line in enumerate(text.splitlines(), 1) if line.rstrip().endswith(PRAGMA)}
    annotations = _annotation_nodes(tree)
    problems = []
    for node in ast.walk(tree):
        message = _problem(node, annotations)
        if message and node.lineno not in allowed:
            problems.append(f'{rel}:{node.lineno} {message} {SUFFIX}')
    return problems


def _annotation_nodes(tree: ast.AST) -> set:
    """Return the ids of every node inside an annotation (never evaluated at runtime)."""
    roots = []
    for node in ast.walk(tree):
        if isinstance(node, (ast.AnnAssign, ast.arg)) and node.annotation is not None:
            roots.append(node.annotation)
        elif isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and node.returns:
            roots.append(node.returns)
    return {id(n) for root in roots for n in ast.walk(root)}


def _problem(node: ast.AST, annotations: set) -> str | None:
    if id(node) in annotations:
        return None
    if isinstance(node, ast.Subscript) and _name(node.value) in GENERICS:
        return f'{_name(node.value)}[...] at runtime: use typing.{_name(node.value).title()}'
    if isinstance(node, (ast.Import, ast.ImportFrom)):
        return _import_problem(node)
    if isinstance(node, ast.Attribute):
        hint = NEW_FUNCTIONS.get((_name(node.value), node.attr))
        return f'{_name(node.value)}.{node.attr}: use {hint}' if hint else None
    if isinstance(node, ast.Call):
        return _call_problem(node)
    return None


def _import_problem(node) -> str | None:
    modules = [a.name for a in node.names] if isinstance(node, ast.Import) else [node.module]
    for module in filter(None, modules):
        hint = NEW_MODULES.get(module.split('.')[0])
        if hint:
            return f'{module} is not on Python 3.8: use {hint}'
    if isinstance(node, ast.ImportFrom) and node.module:
        for alias in node.names:
            hint = NEW_FUNCTIONS.get((node.module, alias.name))
            if hint:
                return f'{node.module}.{alias.name}: use {hint}'
    return None


def _call_problem(node: ast.Call) -> str | None:
    func = node.func
    name = func.attr if isinstance(func, ast.Attribute) else _name(func)
    if isinstance(func, ast.Attribute) and name in NEW_METHODS:
        return f'.{name}(): use {NEW_METHODS[name]}'
    for keyword in node.keywords:
        hint = NEW_KEYWORDS.get((name, keyword.arg))
        if hint:
            return f'{name}({keyword.arg}=): use {hint}'
    if name == 'isinstance' and len(node.args) == 2 and isinstance(node.args[1], ast.BinOp):
        return 'isinstance(x, A | B): use isinstance(x, (A, B))'
    return None


def _name(node: ast.AST) -> str:
    if isinstance(node, ast.Name):
        return node.id
    if isinstance(node, ast.Attribute):
        return f'{_name(node.value)}.{node.attr}'
    return ''
