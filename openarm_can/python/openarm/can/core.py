# Copyright 2025 Enactic, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Core bindings module that imports the C++ extension.
"""

try:
    from openarm_can import *
except ImportError as e:
    raise ImportError(
        "Failed to import openarm_can C++ extension. "
        "Make sure the package is properly installed. "
        f"Original error: {e}"
    ) from e

# Re-export main classes with better names if needed
__all__ = [
    # Enums
    "MotorType",
    "MotorVariable",
    "CallbackMode",

    # Data structures
    "LimitParam",
    "ParamResult",
    "MotorStateResult",
    "CanFrame",
    "CanFdFrame",
    "MITParam",

    # Main classes (1:1 C++ mapping)
    "Motor",
    "MotorControl",
    "CANSocket",
    "CANDevice",
    "MotorDeviceCan",
    "CANDeviceCollection",

    # Exceptions
    "CANSocketException",
]
