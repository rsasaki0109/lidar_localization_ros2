"""Launch arguments that override the parameter YAML only when they are set.

A launch argument left empty does not touch the node: the value comes from the
parameter YAML, or from the fallback when the YAML does not define it. The
resolved value of every argument is also published as ``resolved_<name>`` so
that other actions (static TF publishers) use the same frame as the node.
"""

import os
import tempfile

from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration
import yaml


def _parse(name, text, kind):
    if kind is bool:
        lowered = text.strip().lower()
        if lowered not in ('true', 'false'):
            raise ValueError(f"launch argument {name}:={text!r} must be 'true' or 'false'")
        return lowered == 'true'
    return kind(text)


def _as_text(value):
    return str(value).lower() if isinstance(value, bool) else str(value)


def yaml_parameters(path, node_name):
    """Return the ros__parameters that apply to node_name in a parameter YAML."""
    with open(path, encoding='utf-8') as stream:
        data = yaml.safe_load(stream) or {}
    merged = {}
    for key in ('/**', node_name, '/' + node_name):
        section = data.get(key) or {}
        merged.update(section.get('ros__parameters') or {})
    return merged


def resolve_parameter_overrides(param_file_argument, node_name, specs):
    """Return an action that resolves the overridable launch arguments.

    specs maps an argument name to (type, fallback). The action sets the
    launch configuration ``<node_name>_override_file`` to a parameter file that
    contains only the explicitly set arguments; list it after the parameter
    YAML so that those values win.
    """
    def setup(context):
        from_yaml = yaml_parameters(
            LaunchConfiguration(param_file_argument).perform(context), node_name)
        overrides = {}
        actions = []
        for name, (kind, fallback) in specs.items():
            text = LaunchConfiguration(name).perform(context)
            if text != '':
                value = overrides[name] = _parse(name, text, kind)
            else:
                value = from_yaml.get(name, fallback)
            actions.append(SetLaunchConfiguration('resolved_' + name, _as_text(value)))
        fd, override_path = tempfile.mkstemp(prefix=node_name + '_overrides_', suffix='.yaml')
        with os.fdopen(fd, 'w', encoding='utf-8') as stream:
            yaml.safe_dump({'/**': {'ros__parameters': overrides}}, stream)
        actions.append(SetLaunchConfiguration(node_name + '_override_file', override_path))
        return actions

    return OpaqueFunction(function=setup)
