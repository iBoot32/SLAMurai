"""Nav2 parameters as a nested dict. A change is {path: value}; a path is 'node/ros__parameters/name' (or a profile alias)."""
import copy

import yaml


def load(path):
    return yaml.safe_load(open(path))


def get(d, path):
    for k in path.split('/'):
        d = d[k]
    return d


def put(d, path, value):
    keys = path.split('/')
    for k in keys[:-1]:
        d = d[k]
    if keys[-1] not in d:
        raise KeyError('no such parameter: ' + path)
    d[keys[-1]] = value


def apply(base, changes, aliases=None):
    """Copy of base with the changes applied. Raises KeyError for a path that does not exist."""
    out = copy.deepcopy(base)
    for k, v in changes.items():
        for path in (aliases or {}).get(k, [k]):
            put(out, path, v)
    return out
