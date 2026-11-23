from .static_plugin import StaticPlugin
from .g1.plugin import G1Plugin
from .person.plugin import PersonPlugin

PLUGIN_REGISTRY = {
    "static": StaticPlugin,
    "g1": G1Plugin,
    "person": PersonPlugin,
}