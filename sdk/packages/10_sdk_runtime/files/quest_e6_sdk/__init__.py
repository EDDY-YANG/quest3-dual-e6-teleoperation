"""Public client for the bundled Quest 3 / DOBOT E6 application.

Importing this package performs no network, robot, process or filesystem writes.
The original backend remains the sole owner of robot connections and control.
"""

from .client import SDKError, TeleopClient

__version__ = "0.2.0"
__all__ = ["SDKError", "TeleopClient", "__version__"]
