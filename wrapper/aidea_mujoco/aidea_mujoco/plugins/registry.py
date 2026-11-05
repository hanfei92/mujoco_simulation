from .static_plugin import StaticPlugin
from .g1.plugin import G1Plugin

PLUGIN_REGISTRY = {
    "static": StaticPlugin,
    "g1": G1Plugin,
}