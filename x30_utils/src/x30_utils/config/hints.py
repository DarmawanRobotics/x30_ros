"""Type hints of configuration dataclasses on every supported Python (3.8 included)."""

from __future__ import annotations

import ast
import sys
import typing
from typing import Any

# PEP 585 builtins and their typing aliases (subscriptable on 3.8).
_GENERICS = {
    'tuple': typing.Tuple,
    'list': typing.List,
    'dict': typing.Dict,
    'set': typing.Set,
    'frozenset': typing.FrozenSet,
    'type': typing.Type,
}
_UNION = '__x30_union__'
_PREFIX = '__x30_generic_'


class _Rewrite(ast.NodeTransformer):
    """Rewrite ``a | b`` to ``Union[a, b]`` and ``tuple[...]`` to ``Tuple[...]``."""

    def visit_BinOp(self, node: ast.BinOp) -> ast.AST:
        self.generic_visit(node)
        if not isinstance(node.op, ast.BitOr):
            return node
        members = ast.Tuple(elts=[node.left, node.right], ctx=ast.Load())
        return ast.Subscript(
            value=ast.Name(id=_UNION, ctx=ast.Load()),
            slice=_index(members),
            ctx=ast.Load(),
        )

    def visit_Subscript(self, node: ast.Subscript) -> ast.AST:
        self.generic_visit(node)
        if isinstance(node.value, ast.Name) and node.value.id in _GENERICS:
            node.value = ast.Name(id=_PREFIX + node.value.id, ctx=ast.Load())
        return node


def _index(expr: ast.expr) -> ast.AST:
    """Wrap a subscript index the way this Python's AST expects it."""
    return ast.Index(value=expr) if sys.version_info < (3, 9) else expr


def resolve_annotation(text: str, globalns: dict | None = None, localns: dict | None = None):
    """Evaluate an annotation string, accepting PEP 604 / 585 syntax on any Python."""
    tree = ast.fix_missing_locations(_Rewrite().visit(ast.parse(text, mode='eval')))
    names = {_UNION: typing.Union, **{_PREFIX + k: v for k, v in _GENERICS.items()}}
    scope = {**(globalns or {}), **names}
    try:
        return eval(compile(tree, '<annotation>', 'eval'), scope, dict(localns or {}))
    except Exception as error:
        raise TypeError(f'cannot resolve annotation {text!r}: {error}') from error


def type_hints(cls: type) -> dict[str, Any]:
    """Return the resolved annotations of a class and its bases (like get_type_hints)."""
    try:
        return typing.get_type_hints(cls)  # py38: ok
    except TypeError:
        if sys.version_info >= (3, 10):
            raise
    return _resolved_hints(cls)


def _resolved_hints(cls: type) -> dict[str, Any]:
    hints: dict[str, Any] = {}
    for base in reversed(cls.__mro__):
        # The class's own annotations only: on 3.8 base.__annotations__ may be inherited.
        annotations = vars(base).get('__annotations__', {})
        globalns = getattr(sys.modules.get(base.__module__), '__dict__', {})
        for name, hint in annotations.items():
            text = isinstance(hint, str)
            hints[name] = resolve_annotation(hint, globalns, vars(base)) if text else hint
    return hints
